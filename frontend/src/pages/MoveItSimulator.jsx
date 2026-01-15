// frontend/src/pages/MoveItSimulator.jsx
import React, { useState, useEffect, useRef } from 'react'
import { useQuery, useMutation } from '@tanstack/react-query'
import toast from 'react-hot-toast'
import {
    Play, Square, Save, Maximize2, Settings, Camera,
    Map as MapIcon, Navigation, Target, Zap, Activity,
    Box, Move, RotateCcw, Anchor
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import RvizPanel from '../components/simulation/RvizPanel'
import ModelSelector from '../components/simulation/ModelSelector'
import ScenarioSelector from '../components/simulation/ScenarioSelector'
import StatusPanel from '../components/simulation/StatusPanel'
import ErrorBoundary from '../components/ui/ErrorBoundary'
import { simulationAPI, mapAPI } from '../services/api'
import { wsService } from '../services/ws'
import { rosClient } from '../services/rosClient'

const BACKEND_BASE = import.meta?.env?.VITE_API_URL || '/api'

const MoveItSimulatorContent = () => {
    // Keep exact structure logic
    const [selectedModel, setSelectedModel] = useState('so_arm_100')
    const [selectedScenario, setSelectedScenario] = useState('MOVEIT_DEMO')
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [activeTab, setActiveTab] = useState('control')

    // Connection State
    const [connectionStatus, setConnectionStatus] = useState({
        stomp: 'disconnected',
        ros: 'disconnected'
    })
    const [startupPipeline, setStartupPipeline] = useState({
        stage: 'idle',
        message: '',
        progress: 0
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

    // Simplified Connection Logic (Clone of Simulator.jsx's robust pipeline)
    useEffect(() => {
        let cancelled = false
        mountedRef.current = true

        const envRos = import.meta.env?.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
        const envWs = import.meta.env?.VITE_WS_URL || '/ws/robot'
        // Allow env overrides or defaults
        const rosUrl = localStorage.getItem('rosbridge_url') || envRos
        const wsUrl = localStorage.getItem('ws_url') || envWs

        const initConnections = async () => {
            try {
                // STOMP skipped - focusing on ROS for MoveIt direct control
                // But keeping UI consistent
                setConnectionStatus(prev => ({ ...prev, stomp: 'connected' })) // Fake/Pass for UI
            } catch (err) { console.error(err) }

            try {
                setStartupPipeline({ stage: 'connecting', message: 'Connecting to ROS Bridge...', progress: 50 })
                await rosClient.connect(rosUrl)
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, ros: 'connected' }))
                    setStartupPipeline({ stage: 'ready', message: 'Ready', progress: 100 })
                }
            } catch (err) {
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, ros: 'error' }))
                    setStartupPipeline({ stage: 'error', message: 'ROS Connection Failed', progress: 0 })
                    toast.error('ROS Bridge Connection Failed')
                }
            }
        }

        initConnections()
        return () => {
            cancelled = true
            rosClient.disconnect()
        }
    }, [])

    const handleFullscreen = () => {
        if (!document.fullscreenElement) {
            simulatorRef.current?.requestFullscreen()
            setIsFullscreen(true)
        } else {
            document.exitFullscreen?.()
            setIsFullscreen(false)
        }
    }

    // --- MOVEIT ACTIONS ---
    const sendMoveItCommand = (command, payload = {}) => {
        if (!rosClient.isConnected()) {
            toast.error('ROS Not Connected')
            return
        }
        // Minimal wiring: Publish string command to wrapper node
        const topic = '/so_arm/moveit_cmd'
        try {
            console.log(`[MoveIt] Sending command: ${command}`)
            rosClient.publishTopic(topic, 'std_msgs/String', { data: command })
            toast.success(`Sent: ${command}`)
        } catch (e) {
            console.error(e)
            toast.error('Failed to send command')
        }
    }

    const connectionIndicator = (
        <div className="flex items-center gap-3">
            <div className="flex items-center gap-2 text-xs">
                <div className={`w-2 h-2 rounded-full ${connectionStatus.ros === 'connected' ? 'bg-green-500' : 'bg-red-500'}`} title="ROS" />
                <span className="text-gray-400">{connectionStatus.ros === 'connected' ? 'Connected' : 'Offline'}</span>
            </div>
        </div>
    )

    return (
        <PageContainer
            title="SO101 Simulator"
            description="MoveIt Motion Planning & Simulation"
            actions={connectionIndicator}
        >
            <div className="grid grid-cols-1 xl:grid-cols-4 gap-6 h-[calc(100vh-200px)]" ref={simulatorRef}>
                {/* LEFT PANEL */}
                <div className="xl:col-span-1 space-y-6">
                    <ModelSelector selectedModel={selectedModel} onModelChange={setSelectedModel} disabled={true} />
                    <ScenarioSelector selectedScenario={selectedScenario} onScenarioChange={setSelectedScenario} disabled={true} />

                    <div className="bg-gray-900 border border-gray-800 rounded-xl overflow-hidden">
                        <div className="flex border-b border-gray-800">
                            <button className="flex-1 flex items-center justify-center gap-2 p-3 text-sm font-medium bg-blue-600 text-white">
                                <Zap className="w-4 h-4" /> MoveIt Control
                            </button>
                        </div>
                        <div className="p-4 space-y-3">
                            {/* MOVEIT BUTTONS */}
                            <div className="grid grid-cols-2 gap-2">
                                <button
                                    onClick={() => sendMoveItCommand('plan')}
                                    className="p-3 bg-gray-800 hover:bg-gray-700 text-cyan-400 font-bold rounded-lg border border-gray-700 transition-colors flex flex-col items-center gap-1"
                                >
                                    <MapIcon className="w-5 h-5" />
                                    <span>PLAN</span>
                                </button>
                                <button
                                    onClick={() => sendMoveItCommand('execute')}
                                    className="p-3 bg-gray-800 hover:bg-green-900/40 text-green-400 font-bold rounded-lg border border-gray-700 transition-colors flex flex-col items-center gap-1"
                                >
                                    <Play className="w-5 h-5" />
                                    <span>EXECUTE</span>
                                </button>
                            </div>

                            <button
                                onClick={() => sendMoveItCommand('stop')}
                                className="w-full py-3 bg-red-900/30 hover:bg-red-900/50 text-red-500 font-bold rounded-lg border border-red-900/50 transition-colors flex items-center justify-center gap-2"
                            >
                                <Square className="w-4 h-4 fill-current" /> STOP MOTION
                            </button>

                            <div className="grid grid-cols-2 gap-2 mt-2">
                                <button
                                    onClick={() => sendMoveItCommand('home')}
                                    className="p-2 bg-gray-800 hover:bg-gray-700 text-gray-300 text-xs rounded-lg border border-gray-700 flex items-center justify-center gap-2"
                                >
                                    <RotateCcw className="w-3 h-3" /> Home Pose
                                </button>
                                <button
                                    onClick={() => sendMoveItCommand('gripper_test')}
                                    className="p-2 bg-gray-800 hover:bg-gray-700 text-gray-300 text-xs rounded-lg border border-gray-700 flex items-center justify-center gap-2"
                                >
                                    <Box className="w-3 h-3" /> Gripper Test
                                </button>
                            </div>

                            <div className="mt-4 p-3 bg-black/20 rounded border border-gray-800 text-[10px] font-mono text-gray-500">
                                <div className="mb-1 text-gray-400 font-bold">ROS TOPIC: /so_arm/moveit_cmd</div>
                                <div>Payload: std_msgs/String</div>
                                <div>Commands: plan, execute, stop, home</div>
                            </div>
                        </div>
                    </div>
                </div>

                {/* CENTER PANEL (RVIZ) */}
                <div className="xl:col-span-2">
                    <div className="bg-gray-900 border border-gray-800 rounded-xl h-full relative overflow-hidden">
                        <div className="absolute top-4 right-4 z-10 flex gap-2">
                            <button onClick={handleFullscreen} className="p-2 bg-gray-800/80 hover:bg-gray-700/80 text-white rounded-lg">
                                <Maximize2 className="w-4 h-4" />
                            </button>
                        </div>
                        {/* We use RvizPanel which embeds the noVNC iframe */}
                        <RvizPanel />
                    </div>
                </div>

                {/* RIGHT PANEL */}
                <div className="xl:col-span-1 space-y-6">
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4">
                        <h3 className="text-white font-semibold mb-4 flex items-center gap-2">
                            <Activity className="w-4 h-4 text-purple-400" />
                            Status
                        </h3>
                        <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                                <span className="text-gray-500">Planner:</span>
                                <span className="text-white font-mono">OMPL (RRTConnect)</span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-500">Robot:</span>
                                <span className="text-white font-mono">SO-ARM 100</span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-500">Frame:</span>
                                <span className="text-white font-mono">base_link</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </PageContainer>
    )
}

export default function MoveItSimulator() {
    return (
        <ErrorBoundary>
            <MoveItSimulatorContent />
        </ErrorBoundary>
    )
}
