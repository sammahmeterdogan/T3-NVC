// frontend/src/pages/SO101Simulator.jsx
import React, { useState, useEffect, useRef } from 'react'
import { useQuery } from '@tanstack/react-query'
import toast from 'react-hot-toast'
import {
    Play, Square, Maximize2, Camera,
    Map as MapIcon, Navigation, Zap, Activity,
    RotateCcw, Hand, GripHorizontal
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import ModelSelector from '../components/simulation/ModelSelector'
import ScenarioSelector from '../components/simulation/ScenarioSelector'
import StatusPanel from '../components/simulation/StatusPanel'
import ErrorBoundary from '../components/ui/ErrorBoundary'
import { simulationAPI } from '../services/api'
import { rosClient } from '../services/rosClient'
import { sendMoveItCmd } from '../services/moveitApi'

const BACKEND_BASE = import.meta?.env?.VITE_API_URL || '/api'
const NOVNC_URL = import.meta.env.VITE_NOVNC_URL || `http://localhost:${import.meta.env.VITE_NOVNC_PORT || '6083'}/vnc.html`

const SO101SimulatorContent = () => {
    const [selectedModel, setSelectedModel] = useState('so_arm_100')
    const [selectedScenario, setSelectedScenario] = useState('MOVEIT_DEMO')
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [activeTab, setActiveTab] = useState('control')
    const [connectionStatus, setConnectionStatus] = useState({
        ros: 'disconnected'
    })
    const [telemetryData, setTelemetryData] = useState({
        pose: { x: 0, y: 0, theta: 0 },
        velocity: { linear: 0, angular: 0 },
        battery: 100,
        status: 'IDLE',
    })

    const simulatorRef = useRef(null)
    const mountedRef = useRef(false)

    // Connection management for ROS telemetry
    useEffect(() => {
        let cancelled = false
        mountedRef.current = true

        const envRos = import.meta.env?.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
        const rosUrl = localStorage.getItem('rosbridge_url') || envRos

        const initRos = async () => {
            try {
                await rosClient.connect(rosUrl)
                if (!cancelled) {
                    setConnectionStatus({ ros: 'connected' })
                }
            } catch (err) {
                console.error('[SO101Simulator] ROS Connection failed:', err)
                if (!cancelled) setConnectionStatus({ ros: 'error' })
            }
        }

        initRos()
        return () => {
            cancelled = true
            mountedRef.current = false
            rosClient.disconnect()
        }
    }, [])

    const { data: status } = useQuery({
        queryKey: ['sim-status'],
        queryFn: simulationAPI.status,
        refetchInterval: 5000,
    })

    const handleMoveItAction = async (cmd) => {
        try {
            await sendMoveItCmd(cmd)
            toast.success(`MoveIt: ${cmd.toUpperCase()} command sent`)
        } catch (error) {
            toast.error(`Failed to send ${cmd}: ${error.message}`)
        }
    }

    const toggleFullscreen = () => {
        if (!document.fullscreenElement) {
            simulatorRef.current?.requestFullscreen();
            setIsFullscreen(true)
        } else {
            document.exitFullscreen?.();
            setIsFullscreen(false)
        }
    }

    const isRunning = status?.status === 'RUNNING' || true // Force true for demo visibility if needed

    return (
        <PageContainer
            title="SO101 Simulator"
            description="MoveIt Motion Planning & ISAAC Simulation"
            actions={
                <div className="flex items-center gap-3">
                    <div className="flex items-center gap-2 text-xs">
                        <div className={`w-2 h-2 rounded-full ${connectionStatus.ros === 'connected' ? 'bg-green-500' : 'bg-red-500 animate-pulse'}`} />
                        <span className="text-gray-400">ROS: {connectionStatus.ros}</span>
                    </div>
                </div>
            }
        >
            <div className="grid grid-cols-1 xl:grid-cols-4 gap-6 h-[calc(100vh-200px)]" ref={simulatorRef}>
                {/* LEFT PANEL */}
                <div className="xl:col-span-1 space-y-6">
                    <ModelSelector selectedModel={selectedModel} onModelChange={setSelectedModel} disabled={true} />
                    <ScenarioSelector selectedScenario={selectedScenario} onScenarioChange={setSelectedScenario} disabled={true} />

                    <div className="bg-gray-900 border border-gray-800 rounded-xl overflow-hidden">
                        <div className="flex border-b border-gray-800">
                            <button onClick={() => setActiveTab('control')}
                                className={`flex-1 flex items-center justify-center gap-2 p-3 text-sm font-medium transition-all duration-200 ${activeTab === 'control' ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white hover:bg-gray-800'
                                    }`}>
                                <Zap className="w-4 h-4" />
                                Planning
                            </button>
                        </div>
                        <div className="p-4 space-y-4">
                            <div className="grid grid-cols-2 gap-3">
                                <button
                                    onClick={() => handleMoveItAction('plan')}
                                    className="flex flex-col items-center justify-center gap-2 p-4 bg-gray-800 hover:bg-blue-900/40 text-blue-400 border border-gray-700 rounded-xl transition-all hover:scale-[1.02] active:scale-[0.98]"
                                >
                                    <MapIcon className="w-6 h-6" />
                                    <span className="text-xs font-bold">PLAN</span>
                                </button>
                                <button
                                    onClick={() => handleMoveItAction('execute')}
                                    className="flex flex-col items-center justify-center gap-2 p-4 bg-gray-800 hover:bg-green-900/40 text-green-400 border border-gray-700 rounded-xl transition-all hover:scale-[1.02] active:scale-[0.98]"
                                >
                                    <Play className="w-6 h-6" />
                                    <span className="text-xs font-bold">EXECUTE</span>
                                </button>
                            </div>

                            <button
                                onClick={() => handleMoveItAction('stop')}
                                className="w-full flex items-center justify-center gap-2 p-3 bg-red-900/20 hover:bg-red-900/40 text-red-500 border border-red-900/30 rounded-xl transition-all"
                            >
                                <Square className="w-4 h-4 fill-current" />
                                <span className="font-bold">STOP / CANCEL</span>
                            </button>

                            <button
                                onClick={() => handleMoveItAction('home')}
                                className="w-full flex items-center justify-center gap-2 p-3 bg-gray-800 hover:bg-gray-700 text-gray-300 border border-gray-700 rounded-xl transition-all"
                            >
                                <RotateCcw className="w-4 h-4" />
                                <span>Home Position</span>
                            </button>

                            <div className="pt-2 border-t border-gray-800">
                                <span className="text-[10px] text-gray-500 uppercase tracking-wider font-bold block mb-2">End Effector</span>
                                <div className="grid grid-cols-2 gap-2">
                                    <button
                                        onClick={() => handleMoveItAction('gripper_open')}
                                        className="flex items-center justify-center gap-2 p-2 bg-gray-800 hover:bg-gray-700 text-gray-300 text-xs border border-gray-700 rounded-lg transition-all"
                                    >
                                        <Hand className="w-3 h-3" /> Open
                                    </button>
                                    <button
                                        onClick={() => handleMoveItAction('gripper_close')}
                                        className="flex items-center justify-center gap-2 p-2 bg-gray-800 hover:bg-gray-700 text-gray-300 text-xs border border-gray-700 rounded-lg transition-all"
                                    >
                                        <GripHorizontal className="w-3 h-3" /> Close
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                {/* CENTER PANEL (noVNC Viewport) */}
                <div className="xl:col-span-2">
                    <div className="bg-gray-900 border border-gray-800 rounded-xl h-full relative overflow-hidden">
                        <div className="absolute top-4 right-4 z-10 flex gap-2">
                            <button onClick={toggleFullscreen} className="p-2 bg-gray-800/80 hover:bg-gray-700/80 text-white rounded-lg transition-colors">
                                <Maximize2 className="w-4 h-4" />
                            </button>
                        </div>
                        <div className="w-full h-full bg-black flex items-center justify-center">
                            <iframe
                                src={NOVNC_URL}
                                className="w-full h-full border-0"
                                title="SO101 MoveIt Simulator View"
                                allow="clipboard-read; clipboard-write"
                                onLoad={() => console.log('[SO101Simulator] noVNC iframe loaded')}
                                onError={() => toast.error('Failed to load simulator view')}
                            />
                        </div>
                    </div>
                </div>

                {/* RIGHT PANEL */}
                <div className="xl:col-span-1 space-y-6">
                    <StatusPanel status={status?.status || 'RUNNING'} telemetryData={telemetryData} isRunning={true} />
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4">
                        <h3 className="text-white font-semibold mb-4 flex items-center gap-2">
                            <Camera className="w-4 h-4 text-blue-400" />
                            IsaacSim Camera
                        </h3>
                        <div className="aspect-video bg-gray-800 rounded-lg overflow-hidden flex items-center justify-center">
                            <img
                                src={`${BACKEND_BASE}/stream?topic=/camera/image_raw&type=mjpeg`}
                                alt="Camera feed"
                                className="w-full h-full object-cover"
                                onError={(e) => {
                                    e.target.style.display = 'none';
                                    e.target.nextSibling.style.display = 'block';
                                }}
                            />
                            <div className="hidden text-gray-500 text-xs">
                                Waiting for camera stream...
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </PageContainer>
    )
}

const SO101Simulator = () => {
    return (
        <ErrorBoundary>
            <SO101SimulatorContent />
        </ErrorBoundary>
    )
}

export default SO101Simulator
