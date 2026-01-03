// frontend/src/pages/Simulator.jsx
import React, { useState, useEffect, useRef } from 'react'
import { useQuery, useMutation } from '@tanstack/react-query'
import toast from 'react-hot-toast'
import {
    Play, Square, Save, Maximize2, Settings, Camera,
    Map as MapIcon, Navigation, Target, Zap, Activity,
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import RvizPanel from '../components/simulation/RvizPanel'
import TeleopPad from '../components/simulation/TeleopPad'
import ModelSelector from '../components/simulation/ModelSelector'
import ScenarioSelector from '../components/simulation/ScenarioSelector'
import StatusPanel from '../components/simulation/StatusPanel'
import ErrorBoundary from '../components/ui/ErrorBoundary'
import { simulationAPI, mapAPI } from '../services/api'
import { wsService } from '../services/ws'
import { rosClient } from '../services/rosClient'

const toEnum = (val) => String(val ?? '').trim().replace(/[-\s]+/g, '_').toUpperCase()
const BACKEND_BASE = import.meta?.env?.VITE_API_URL || '/api'

const SimulatorContent = () => {
    const [selectedModel, setSelectedModel] = useState('burger')
    const [selectedScenario, setSelectedScenario] = useState('TELEOP')
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [activeTab, setActiveTab] = useState('control')
    const [connectionStatus, setConnectionStatus] = useState({
        stomp: 'disconnected',
        ros: 'disconnected'
    })
    const [telemetryData, setTelemetryData] = useState({
        pose: { x: 0, y: 0, theta: 0 },
        velocity: { linear: 0, angular: 0 },
        battery: 100,
        status: 'IDLE',
    })

    const simulatorRef = useRef(null)
    const subscriptionsRef = useRef([])
    const mountedRef = useRef(false)

    // Connection management effect - runs once on mount
    useEffect(() => {
        let cancelled = false
        mountedRef.current = true
        
        const envRos = import.meta.env?.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
        const envWs = import.meta.env?.VITE_WS_URL || '/ws/robot'
        const rosUrl = localStorage.getItem('rosbridge_url') || envRos
        const wsUrl = localStorage.getItem('ws_url') || envWs

        console.log('[Simulator] Initializing connections...')

        const initConnections = async () => {
            try {
                // Connect STOMP first
                console.log('[Simulator] Connecting to STOMP...')
                await wsService.connect(wsUrl)
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, stomp: 'connected' }))
                    console.log('[Simulator] STOMP connected, setting up subscriptions...')
                    setupStompSubscriptions()
                }
            } catch (err) {
                console.error('[Simulator] STOMP connection failed:', err)
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, stomp: 'error' }))
                }
            }

            try {
                // Connect ROS Bridge
                console.log('[Simulator] Connecting to ROS Bridge...')
                await rosClient.connect(rosUrl)
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, ros: 'connected' }))
                    console.log('[Simulator] ROS Bridge connected')
                }
            } catch (err) {
                console.error('[Simulator] ROS Bridge connection failed:', err)
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, ros: 'error' }))
                }
            }
        }

        const setupStompSubscriptions = () => {
            if (!wsService.isConnected()) {
                console.warn('[Simulator] STOMP not connected, skipping subscriptions')
                return
            }

            // Telemetry subscription
            const telemetryUnsub = wsService.subscribe('/topic/telemetry', (data) => {
                if (mountedRef.current) {
                    setTelemetryData((prev) => ({ ...prev, ...data }))
                }
            })

            // Status subscription
            const statusUnsub = wsService.subscribe('/topic/status', () => {
                if (mountedRef.current) {
                    refetchStatus()
                }
            })

            subscriptionsRef.current.push(telemetryUnsub, statusUnsub)
            console.log('[Simulator] STOMP subscriptions created')
        }

        initConnections()

        // Cleanup on unmount
        return () => {
            console.log('[Simulator] Cleaning up connections...')
            cancelled = true
            mountedRef.current = false

            // Unsubscribe all STOMP subscriptions
            subscriptionsRef.current.forEach(unsub => {
                if (typeof unsub === 'function') {
                    try {
                        unsub()
                    } catch (e) {
                        console.warn('[Simulator] Subscription cleanup error:', e)
                    }
                }
            })
            subscriptionsRef.current = []

            // Disconnect services
            wsService.disconnect()
            rosClient.disconnect()
        }
    }, []) // Empty deps - run once on mount

    const { data: status, refetch: refetchStatus } = useQuery({
        queryKey: ['sim-status'],
        queryFn: simulationAPI.status,
        refetchInterval: 2000,
    })

    const startSimulation = useMutation({
        mutationFn: () => simulationAPI.start({ model: toEnum(selectedModel), scenario: toEnum(selectedScenario) }),
        onSuccess: () => { toast.success('Simulation started successfully'); refetchStatus() },
        onError: (error) => { toast.error('Failed to start simulation'); console.error(error) },
    })

    const stopSimulation = useMutation({
        mutationFn: simulationAPI.stop,
        onSuccess: () => { toast.success('Simulation stopped'); refetchStatus() },
        onError: (error) => { toast.error('Failed to stop simulation'); console.error(error) },
    })

    const saveMap = useMutation({
        mutationFn: () => mapAPI.save({ name: `map_${Date.now()}` }),
        onSuccess: () => toast.success('Map saved successfully'),
        onError: (error) => { toast.error('Failed to save map'); console.error(error) },
    })

    // ROS odometry subscription - only when simulation is running
    useEffect(() => {
        if (status?.status !== 'RUNNING') return
        if (!rosClient.isConnected()) {
            console.warn('[Simulator] ROS not connected, skipping odom subscription')
            return
        }

        let cancelled = false

        const setupOdomSubscription = () => {
            try {
                console.log('[Simulator] Setting up odometry subscription')
                const odomTopic = rosClient.subscribeTopic('/odom', 'nav_msgs/Odometry', (msg) => {
                    if (cancelled || !mountedRef.current) return

                    const pose = msg?.pose?.pose?.position || { x: 0, y: 0 }
                    const q = msg?.pose?.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 }
                    const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    
                    setTelemetryData((prev) => ({
                        ...prev,
                        pose: { 
                            x: pose.x || 0, 
                            y: pose.y || 0, 
                            theta: Number.isFinite(theta) ? theta : 0 
                        },
                    }))
                })

                return () => {
                    cancelled = true
                    try {
                        if (odomTopic) {
                            console.log('[Simulator] Cleaning up odometry subscription')
                            rosClient.unsubscribeTopic('/odom')
                        }
                    } catch (e) {
                        console.error('[Simulator] Odom unsubscribe error:', e)
                    }
                }
            } catch (err) {
                console.error('[Simulator] Failed to setup odom subscription:', err)
            }
        }

        return setupOdomSubscription()
    }, [status?.status])

    const toggleFullscreen = () => {
        if (!document.fullscreenElement) { 
            simulatorRef.current?.requestFullscreen(); 
            setIsFullscreen(true) 
        } else { 
            document.exitFullscreen?.(); 
            setIsFullscreen(false) 
        }
    }

    const handlePublishGoal = () => {
        if (!rosClient.isConnected()) {
            toast.error('ROS Bridge not connected')
            return
        }

        try {
            rosClient.publishTopic('/move_base_simple/goal', 'geometry_msgs/PoseStamped', {
                header: { frame_id: 'map' },
                pose: { 
                    position: { x: 1.0, y: 0.0, z: 0 }, 
                    orientation: { x: 0, y: 0, z: 0, w: 1 } 
                },
            })
            toast.success('Goal published')
        } catch (e) {
            console.error('[Simulator] Goal publish failed:', e)
            toast.error('Goal publish failed')
        }
    }

    const handleCancelNav = () => {
        if (!rosClient.isConnected()) {
            toast.error('ROS Bridge not connected')
            return
        }

        try {
            rosClient.publishTopic('/move_base/cancel', 'actionlib_msgs/GoalID', {
                stamp: { secs: 0, nsecs: 0 }, 
                id: ''
            })
            toast.success('Navigation cancelled')
        } catch (e) {
            console.error('[Simulator] Cancel nav failed:', e)
        }
    }

    const isRunning = status?.status === 'RUNNING'

    // Connection status indicator
    const connectionIndicator = (
        <div className="flex items-center gap-2 text-xs">
            <div className={`w-2 h-2 rounded-full ${connectionStatus.stomp === 'connected' ? 'bg-green-500' : connectionStatus.stomp === 'error' ? 'bg-red-500' : 'bg-yellow-500 animate-pulse'}`} title="STOMP" />
            <div className={`w-2 h-2 rounded-full ${connectionStatus.ros === 'connected' ? 'bg-green-500' : connectionStatus.ros === 'error' ? 'bg-red-500' : 'bg-yellow-500 animate-pulse'}`} title="ROS" />
            <span className="text-gray-400">
                {connectionStatus.stomp === 'connected' && connectionStatus.ros === 'connected' ? 'Connected' : 
                 connectionStatus.stomp === 'error' || connectionStatus.ros === 'error' ? 'Connection error' : 
                 'Connecting...'}
            </span>
        </div>
    )

    return (
        <PageContainer
            title="TurtleBot3 Simulator"
            description="Real-time robot simulation and control"
            actions={
                <div className="flex items-center gap-3">
                    {connectionIndicator}
                    {!isRunning ? (
                        <button 
                            onClick={() => startSimulation.mutate()} 
                            disabled={startSimulation.isPending}
                            className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95"
                        >
                            <Play className="w-4 h-4" />
                            {startSimulation.isPending ? 'Starting...' : 'Start Simulation'}
                        </button>
                    ) : (
                        <button 
                            onClick={() => stopSimulation.mutate()} 
                            disabled={stopSimulation.isPending}
                            className="flex items-center gap-2 px-4 py-2 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95"
                        >
                            <Square className="w-4 h-4" />
                            {stopSimulation.isPending ? 'Stopping...' : 'Stop Simulation'}
                        </button>
                    )}
                    <button 
                        onClick={() => saveMap.mutate()} 
                        disabled={!isRunning || saveMap.isPending}
                        className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95"
                    >
                        <Save className="w-4 h-4" />
                        {saveMap.isPending ? 'Saving...' : 'Save Map'}
                    </button>
                </div>
            }
        >
            <div className="grid grid-cols-1 xl:grid-cols-4 gap-6 h-[calc(100vh-200px)]" ref={simulatorRef}>
                <div className="xl:col-span-1 space-y-6">
                    <ModelSelector selectedModel={selectedModel} onModelChange={setSelectedModel} disabled={isRunning} />
                    <ScenarioSelector selectedScenario={selectedScenario} onScenarioChange={setSelectedScenario} disabled={isRunning} />
                    <div className="bg-gray-900 border border-gray-800 rounded-xl overflow-hidden">
                        <div className="flex border-b border-gray-800">
                            {[
                                { id: 'control', label: 'Control', icon: Zap },
                                { id: 'navigation', label: 'Navigation', icon: Navigation },
                                { id: 'mapping', label: 'Mapping', icon: MapIcon },
                            ].map((tab) => {
                                const Icon = tab.icon
                                return (
                                    <button key={tab.id} onClick={() => setActiveTab(tab.id)}
                                            className={`flex-1 flex items-center justify-center gap-2 p-3 text-sm font-medium transition-all duration-200 ${
                                                activeTab === tab.id ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white hover:bg-gray-800'
                                            }`}>
                                        <Icon className="w-4 h-4" />
                                        {tab.label}
                                    </button>
                                )
                            })}
                        </div>
                        <div className="p-4">
                            {activeTab === 'control' && (
                                <TeleopPad
                                    enabled={isRunning && connectionStatus.ros === 'connected'}
                                    disabled={!isRunning || connectionStatus.ros !== 'connected'}
                                />
                            )}

                            {activeTab === 'navigation' && (
                                <div className="space-y-3">
                                    <button 
                                        className="w-full py-2 px-4 bg-gray-800 hover:bg-gray-700 disabled:bg-gray-800 disabled:opacity-50 disabled:cursor-not-allowed text-white rounded-lg transition-colors"
                                        onClick={handlePublishGoal}
                                        disabled={connectionStatus.ros !== 'connected'}
                                    >
                                        <Target className="w-4 h-4 inline mr-2" />
                                        Set Goal (1,0)
                                    </button>
                                    <button 
                                        className="w-full py-2 px-4 bg-gray-800 hover:bg-gray-700 disabled:bg-gray-800 disabled:opacity-50 disabled:cursor-not-allowed text-white rounded-lg transition-colors"
                                        onClick={handleCancelNav}
                                        disabled={connectionStatus.ros !== 'connected'}
                                    >
                                        Cancel Navigation
                                    </button>
                                </div>
                            )}

                            {activeTab === 'mapping' && (
                                <div className="space-y-3">
                                    <button className="w-full py-2 px-4 bg-gray-800 text-white rounded-lg opacity-50 cursor-not-allowed" disabled>
                                        Start SLAM
                                    </button>
                                    <button className="w-full py-2 px-4 bg-gray-800 text-white rounded-lg opacity-50 cursor-not-allowed" disabled>
                                        Stop SLAM
                                    </button>
                                </div>
                            )}
                        </div>
                    </div>
                </div>

                <div className="xl:col-span-2">
                    <div className="bg-gray-900 border border-gray-800 rounded-xl h-full relative overflow-hidden">
                        <div className="absolute top-4 right-4 z-10 flex gap-2">
                            <button onClick={toggleFullscreen} className="p-2 bg-gray-800/80 hover:bg-gray-700/80 text-white rounded-lg">
                                <Maximize2 className="w-4 h-4" />
                            </button>
                            <button className="p-2 bg-gray-800/80 hover:bg-gray-700/80 text-white rounded-lg">
                                <Settings className="w-4 h-4" />
                            </button>
                        </div>
                        <RvizPanel />
                    </div>
                </div>

                <div className="xl:col-span-1 space-y-6">
                    <StatusPanel status={status?.status} telemetryData={telemetryData} isRunning={isRunning} />
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4">
                        <h3 className="text-white font-semibold mb-4 flex items-center gap-2">
                            <Camera className="w-4 h-4 text-blue-400" />
                            Camera Feed
                        </h3>
                        <div className="aspect-video bg-gray-800 rounded-lg overflow-hidden">
                            {isRunning ? (
                                <img
                                    src={`${BACKEND_BASE}/stream?topic=/camera/image_raw&type=mjpeg`}
                                    alt="Camera feed"
                                    className="w-full h-full object-cover"
                                    onError={(e) => { if (e.currentTarget) e.currentTarget.src = '/placeholder-camera.jpg' }}
                                />
                            ) : (
                                <div className="flex items-center justify-center h-full">
                                    <p className="text-gray-400 text-sm">Camera feed inactive</p>
                                </div>
                            )}
                        </div>
                    </div>
                </div>
            </div>
        </PageContainer>
    )
}

const Simulator = () => {
    const handleRetry = () => {
        console.log('[Simulator] Retry requested, reloading...')
        window.location.reload()
    }

    return (
        <ErrorBoundary onRetry={handleRetry}>
            <SimulatorContent />
        </ErrorBoundary>
    )
}

export default Simulator
