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
import { simulationAPI, mapAPI } from '../services/api'
import { wsService } from '../services/ws'
import { rosClient } from '../services/rosClient'

const toEnum = (val) => String(val ?? '').trim().replace(/[-\s]+/g, '_').toUpperCase()
const BACKEND_BASE = import.meta?.env?.VITE_API_URL || '/api'

const Simulator = () => {
    const [selectedModel, setSelectedModel] = useState('burger')
    const [selectedScenario, setSelectedScenario] = useState('TELEOP')
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [activeTab, setActiveTab] = useState('control')
    const [telemetryData, setTelemetryData] = useState({
        pose: { x: 0, y: 0, theta: 0 },
        velocity: { linear: 0, angular: 0 },
        battery: 100,
        status: 'IDLE',
    })

    const simulatorRef = useRef(null)

    useEffect(() => {
        // Başlangıç bağlantıları (env veya localStorage override’larına saygı)
        const envRos = import.meta.env?.VITE_ROSBRIDGE_URL || '/rosbridge'
        const envWs  = import.meta.env?.VITE_WS_URL || '/ws/robot'
        const rosUrl = localStorage.getItem('rosbridge_url') || envRos
        const wsUrl  = localStorage.getItem('ws_url') || envWs
        wsService.connect(wsUrl).catch(() => {})
        rosClient.connect(rosUrl).catch(() => {})
        return () => {
            wsService.disconnect()
            rosClient.disconnect()
        }
    }, [])

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

    useEffect(() => {
        const telemetrySub = wsService.subscribe('/topic/telemetry', (data) => {
            setTelemetryData((prev) => ({ ...prev, ...data }))
        })
        const statusSub = wsService.subscribe('/topic/status', () => { refetchStatus() })
        return () => {
            try { if (telemetrySub) wsService.unsubscribe('/topic/telemetry') } catch {}
            try { if (statusSub) wsService.unsubscribe('/topic/status') } catch {}
        }
    }, [refetchStatus])

    useEffect(() => {
        if (status?.status === 'RUNNING') {
            const odomTopic = rosClient.subscribeTopic('/odom', 'nav_msgs/Odometry', (msg) => {
                const pose = msg?.pose?.pose?.position || { x: 0, y: 0 }
                const q = msg?.pose?.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 }
                const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                setTelemetryData((prev) => ({
                    ...prev,
                    pose: { x: pose.x || 0, y: pose.y || 0, theta: Number.isFinite(theta) ? theta : 0 },
                }))
            })
            return () => {
                try { if (odomTopic) rosClient.unsubscribeTopic('/odom') } catch (e) { console.error('Unsubscribe error:', e) }
            }
        }
    }, [status?.status])

    const toggleFullscreen = () => {
        if (!document.fullscreenElement) { simulatorRef.current?.requestFullscreen(); setIsFullscreen(true) }
        else { document.exitFullscreen?.(); setIsFullscreen(false) }
    }

    const isRunning = status?.status === 'RUNNING'

    return (
        <PageContainer
            title="TurtleBot3 Simulator"
            description="Real-time robot simulation and control"
            actions={
                <div className="flex items-center gap-3">
                    {!isRunning ? (
                        <button onClick={() => startSimulation.mutate()} disabled={startSimulation.isLoading}
                                className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95">
                            <Play className="w-4 h-4" />
                            {startSimulation.isLoading ? 'Starting...' : 'Start Simulation'}
                        </button>
                    ) : (
                        <button onClick={() => stopSimulation.mutate()} disabled={stopSimulation.isLoading}
                                className="flex items-center gap-2 px-4 py-2 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95">
                            <Square className="w-4 h-4" />
                            {stopSimulation.isLoading ? 'Stopping...' : 'Stop Simulation'}
                        </button>
                    )}
                    <button onClick={() => saveMap.mutate()} disabled={!isRunning || saveMap.isLoading}
                            className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95">
                        <Save className="w-4 h-4" />
                        {saveMap.isLoading ? 'Saving...' : 'Save Map'}
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
                                    enabled={isRunning}
                                    disabled={!isRunning}
                                />
                            )}

                            {activeTab === 'navigation' && (
                                <div className="space-y-3">
                                    <button className="w-full py-2 px-4 bg-gray-800 hover:bg-gray-700 text-white rounded-lg transition-colors"
                                            onClick={() => {
                                                try {
                                                    rosClient.publishTopic('/move_base_simple/goal', 'geometry_msgs/PoseStamped', {
                                                        header: { frame_id: 'map' },
                                                        pose: { position: { x: 1.0, y: 0.0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
                                                    })
                                                } catch (e) {
                                                    console.error(e)
                                                    toast.error('Goal publish failed')
                                                }
                                            }}>
                                        <Target className="w-4 h-4 inline mr-2" />
                                        Set Goal (1,0)
                                    </button>
                                    <button className="w-full py-2 px-4 bg-gray-800 hover:bg-gray-700 text-white rounded-lg transition-colors"
                                            onClick={() => {
                                                try {
                                                    rosClient.publishTopic('/move_base/cancel', 'actionlib_msgs/GoalID', {
                                                        stamp: { secs: 0, nsecs: 0 }, id: ''
                                                    })
                                                } catch (e) { console.error(e) }
                                            }}>
                                        Cancel Navigation
                                    </button>
                                </div>
                            )}

                            {activeTab === 'mapping' && (
                                <div className="space-y-3">
                                    <button className="w-full py-2 px-4 bg-gray-800 text-white rounded-lg" disabled>Start SLAM (wire API)</button>
                                    <button className="w-full py-2 px-4 bg-gray-800 text-white rounded-lg" disabled>Stop SLAM (wire API)</button>
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

export default Simulator
