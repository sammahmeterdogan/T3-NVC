
import React, { useState, useEffect, useRef, useCallback } from 'react'
import toast from 'react-hot-toast'
import {
    Settings, Maximize2, Activity,
    Home, Grip, Usb, Wifi, Loader
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import RvizPanel from '../components/simulation/RvizPanel'
import ArmControls from '../components/arm/ArmControls'
import StatusPanel from '../components/simulation/StatusPanel'
import LiveRobotScene from '../components/simulation/LiveRobotScene'
import ErrorBoundary from '../components/ui/ErrorBoundary'
import { wsService } from '../services/ws'
import { rosClient } from '../services/rosClient'
import { armAPI } from '../services/api'
import { webSerialService, SCS } from '../services/webSerial'

const SoArm101Content = () => {
    const [jointControlMode, setJointControlMode] = useState(() => {
        return localStorage.getItem('soarm101_joint_control_mode') || 'basic'
    })
    const [controlMode, setControlMode] = useState(() => {
        return localStorage.getItem('soarm101_mode') || 'JOINTS' // JOINTS | CARTESIAN
    })

    // --- LIFTED STATE SOARM 101 ---
    const [joints, setJoints] = useState({
        base: 0,
        shoulder: 0,
        elbow: 0,
        wristPitch: 0,
        wristRoll: 0,
        gripper: 50
    })

    const [cartesianTarget, setCartesianTarget] = useState({
        x: 0.0, y: 0.0, z: 0.0,
    })
    const [connectionStatus, setConnectionStatus] = useState({
        stomp: 'disconnected',
        ros: 'disconnected'
    })
    const [startupPipeline, setStartupPipeline] = useState({
        stage: 'idle', // idle | connecting | subscribing | ready | error
        message: '',
        progress: 0 // 0-100
    })
    const [telemetryData, setTelemetryData] = useState({
        pose: { x: 0, y: 0, theta: 0 },
        velocity: { linear: 0, angular: 0 },
        battery: 100,
        status: 'IDLE',
    })

    // --- HYBRID CONNECTION STATE ---
    const [connectionMode, setConnectionMode] = useState('ROS') // 'ROS' | 'DIRECT'
    const [directStatus, setDirectStatus] = useState({
        connected: false,
        portLabel: ''
    })
    const [isSerialSupported, setIsSerialSupported] = useState(false)

    const containerRef = useRef(null)
    const subscriptionsRef = useRef([])
    const mountedRef = useRef(false)

    // Throttling Reference
    const lastNetworkUpdateRef = useRef(0)
    const NETWORK_THROTTLE_MS = 100 // Limit updates to 10Hz

    // Persist control mode preference
    useEffect(() => {
        localStorage.setItem('soarm101_joint_control_mode', jointControlMode)
    }, [jointControlMode])

    useEffect(() => {
        localStorage.setItem('soarm101_mode', controlMode)
    }, [controlMode])

    // Check Serial Support
    useEffect(() => {
        setIsSerialSupported(webSerialService.isSupported())
    }, [])

    // Connection management effect - runs once on mount
    useEffect(() => {
        let cancelled = false
        mountedRef.current = true

        const envRos = import.meta.env?.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
        const envWs = import.meta.env?.VITE_WS_URL || '/ws/robot'
        const rosUrl = localStorage.getItem('rosbridge_url') || envRos
        const wsUrl = localStorage.getItem('ws_url') || envWs

        const initConnections = async () => {
            try {
                // Stage 1: Connecting to STOMP
                setStartupPipeline({ stage: 'connecting', message: 'Connecting to backend WebSocket...', progress: 20 })

                await wsService.connect(wsUrl)

                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, stomp: 'connected' }))
                    setStartupPipeline({ stage: 'connecting', message: 'Backend WebSocket connected', progress: 40 })
                }
            } catch (err) {
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, stomp: 'error' }))
                    toast.error('STOMP connection failed')
                }
            }

            try {
                // Stage 2: Connecting to ROS Bridge
                setStartupPipeline({ stage: 'connecting', message: 'Connecting to ROS Bridge...', progress: 50 })

                await rosClient.connect(rosUrl)

                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, ros: 'connected' }))
                    setStartupPipeline({ stage: 'connecting', message: 'ROS Bridge connected', progress: 70 })
                }
            } catch (err) {
                if (!cancelled) {
                    setConnectionStatus(prev => ({ ...prev, ros: 'error' }))
                    toast.error('ROS Bridge connection failed')
                }
            }

            // Stage 3: Setting up subscriptions
            if (!cancelled && wsService.isConnected()) {
                setStartupPipeline({ stage: 'subscribing', message: 'Setting up data streams...', progress: 85 })
                setupStompSubscriptions()
            }

            // Stage 4: Ready
            if (!cancelled) {
                setStartupPipeline({ stage: 'ready', message: 'So Arm101 ready', progress: 100 })

                setTimeout(() => {
                    if (mountedRef.current) {
                        setStartupPipeline(prev =>
                            prev.stage === 'ready' ? { ...prev, message: '' } : prev
                        )
                    }
                }, 2000)
            }
        }

        const setupStompSubscriptions = () => {
            if (!wsService.isConnected()) return
            const telemetryUnsub = wsService.subscribe('/topic/telemetry', (data) => {
                if (mountedRef.current) {
                    setTelemetryData((prev) => ({ ...prev, ...data }))
                }
            })
            subscriptionsRef.current.push(telemetryUnsub)
        }

        initConnections()

        // Cleanup on unmount
        return () => {
            cancelled = true
            mountedRef.current = false

            subscriptionsRef.current.forEach(unsub => unsub?.())
            subscriptionsRef.current = []

            wsService.disconnect()
            rosClient.disconnect()
            webSerialService.disconnect()
        }
    }, [])

    const toggleFullscreen = () => {
        if (!document.fullscreenElement) {
            containerRef.current?.requestFullscreen()
        } else {
            document.exitFullscreen?.()
        }
    }

    // --- HYBRID GATEWAY LOGIC ---
    const handleModeSwitch = (mode) => {
        setConnectionMode(mode)
        if (mode === 'DIRECT') {
            toast('Ensure Python Driver is STOPPED to prevent conflicts', {
                icon: '⚠️',
                duration: 5000
            })
        }
    }

    const handleUsbConnect = async () => {
        if (directStatus.connected) {
            await webSerialService.disconnect()
            setDirectStatus({ connected: false, portLabel: '' })
            toast.success('USB Disconnected')
        } else {
            try {
                const result = await webSerialService.connect()
                setDirectStatus({ connected: true, portLabel: result.portName })
                toast.success('USB Connected')
                setConnectionMode('DIRECT')
            } catch (err) {
                toast.error('Connection Failed: ' + err.message)
            }
        }
    }

    // --- CENTRAL COMMAND GATEWAY (THROTTLED) ---
    const handleJointUpdate = useCallback((rawJoints) => {
        // 0. Validation: Ensure no NaNs to prevent crashes in 3D scene or Driver
        const newJoints = {
            base: Number(rawJoints.base) || 0,
            shoulder: Number(rawJoints.shoulder) || 0,
            elbow: Number(rawJoints.elbow) || 0,
            wristPitch: Number(rawJoints.wristPitch) || 0,
            wristRoll: Number(rawJoints.wristRoll) || 0,
            gripper: Number(rawJoints.gripper) || 50
        }

        // 1. Update React State INSTANTLY (for UI / 3D Scene)
        setJoints(newJoints)

        // 2. Throttle Network calls
        const now = Date.now()
        if (now - lastNetworkUpdateRef.current < NETWORK_THROTTLE_MS) {
            return
        }
        lastNetworkUpdateRef.current = now

        // 3. Send over Network
        if (connectionMode === 'DIRECT' && directStatus.connected) {
            /**
             * STS3215 SERVO MAPPING
             * Convert degrees to servo steps (0-4095)
             * Center position = 2048 (corresponds to 0 degrees)
             * 
             * Joint ID Mapping (Physical Servo Bus):
             *   ID 1 = Base (shoulder_pan)
             *   ID 2 = Shoulder (shoulder_lift)
             *   ID 3 = Elbow (elbow_flex)
             *   ID 4 = WristPitch (wrist_flex)
             *   ID 5 = WristRoll (wrist_roll)
             *   ID 6 = Gripper
             */
            const servoCommands = [
                { id: 1, position: webSerialService.degreesToSteps(newJoints.base, -180, 180) },
                { id: 2, position: webSerialService.degreesToSteps(newJoints.shoulder, -90, 90) },
                { id: 3, position: webSerialService.degreesToSteps(newJoints.elbow, -135, 135) },
                { id: 4, position: webSerialService.degreesToSteps(newJoints.wristPitch, -90, 90) },
                { id: 5, position: webSerialService.degreesToSteps(newJoints.wristRoll, -180, 180) },
                { id: 6, position: Math.floor((newJoints.gripper / 100) * SCS.MAX_POSITION) }  // Gripper: 0-100% -> 0-4095
            ]

            // Debug output
            console.log('[STS3215] Joint → Step mapping:', servoCommands.map(s => `ID${s.id}:${s.position}`).join(', '))

            // Send sync write (all servos move simultaneously)
            webSerialService.syncWritePositions(servoCommands, 200).catch(err => {
                if (err.name === 'NetworkError') {
                    setDirectStatus({ connected: false, portLabel: '' })
                    toast.error('Device disconnected')
                }
            })
        } else if (connectionMode === 'ROS' && rosClient.isConnected()) {
            armAPI.sendJoints(newJoints).catch(() => {
                toast.error('Failed to send joint command')
            })
        }
    }, [connectionMode, directStatus.connected])

    const handleHomePosition = () => {
        const homeJoints = { base: 0, shoulder: 0, elbow: 0, wristPitch: 0, wristRoll: 0, gripper: 50 }
        handleJointUpdate(homeJoints) // This handles state update + network (if throttled allowed)

        // Force network send just in case throttle blocked it
        if (connectionMode === 'ROS' && rosClient.isConnected()) {
            rosClient.publishTopic('/so_arm101/home', 'std_msgs/Empty', {})
        }
        toast.success('Home Position Command Sent')
    }

    /**
     * KICKSTART TEST - Send servo ID 1 to center position (2048)
     * Use this to verify serial connection works at the hardware level
     */
    const handleSerialKickstart = async () => {
        if (!directStatus.connected) {
            toast.error('Connect USB first!')
            return
        }

        try {
            // Test single servo: ID 1 to center position
            console.log('[STS3215] ⚡ KICKSTART TEST: Sending ID:1 to center (2048)...')
            await webSerialService.setPosition(1, SCS.CENTER_POSITION, 1000)
            toast.success('⚡ KICKSTART: Servo ID 1 → Center (2048)')
        } catch (err) {
            toast.error('Kickstart failed: ' + err.message)
        }
    }

    const isControlEnabled = connectionMode === 'DIRECT' ? directStatus.connected : (connectionStatus.ros === 'connected')

    const handleCartesianInputChange = (axis, value) => {
        const num = Number(value)
        setCartesianTarget((prev) => ({
            ...prev,
            [axis]: Number.isNaN(num) ? 0 : num,
        }))
    }

    const handleMoveToPosition = async () => {
        if (!isControlEnabled) {
            toast.error('Not connected')
            return
        }

        const { x, y, z } = cartesianTarget

        if (connectionMode === 'DIRECT') {
            toast.error('IK Solver not available in Direct Mode')
            return
        }

        toast(`Calculating IK solution for [${x}, ${y}, ${z}]...`)
        try {
            await armAPI.sendPose({ x, y, z, pitch: 0, roll: 0, yaw: 0 })
            toast.success('Move Executed')
        } catch (e) {
            toast.error('IK move failed: ' + (e.message || 'Unknown error'))
        }
    }

    // Connection status indicator with startup pipeline
    const connectionIndicator = (
        <div className="flex items-center gap-3">
            {connectionMode === 'ROS' ? (
                <>
                    <div className="flex items-center gap-2 text-xs">
                        <div className={`w-2 h-2 rounded-full ${connectionStatus.stomp === 'connected' ? 'bg-green-500' : connectionStatus.stomp === 'error' ? 'bg-red-500' : 'bg-yellow-500 animate-pulse'}`} title="STOMP" />
                        <div className={`w-2 h-2 rounded-full ${connectionStatus.ros === 'connected' ? 'bg-green-500' : connectionStatus.ros === 'error' ? 'bg-red-500' : 'bg-yellow-500 animate-pulse'}`} title="ROS" />
                        <span className="text-gray-400">
                            {startupPipeline.stage === 'ready' ? 'Ready' :
                                startupPipeline.stage === 'error' ? 'Error' :
                                    startupPipeline.stage === 'connecting' || startupPipeline.stage === 'subscribing' ? 'Connecting' :
                                        'Idle'}
                        </span>
                    </div>
                    {startupPipeline.message && startupPipeline.stage !== 'idle' && (
                        <div className="flex items-center gap-2">
                            {startupPipeline.stage === 'connecting' || startupPipeline.stage === 'subscribing' ? (
                                <div className="w-16 h-1 bg-gray-700 rounded-full overflow-hidden">
                                    <div
                                        className="h-full bg-blue-500 transition-all duration-300"
                                        style={{ width: `${startupPipeline.progress}%` }}
                                    />
                                </div>
                            ) : null}
                        </div>
                    )}
                </>
            ) : (
                <div className="flex items-center gap-2 text-xs">
                    <div className={`w-2 h-2 rounded-full ${directStatus.connected ? 'bg-green-500' : 'bg-gray-500'}`} title="USB" />
                    <span className={directStatus.connected ? 'text-green-400' : 'text-gray-400'}>
                        {directStatus.connected ? 'USB Active' : 'USB Disconnected'}
                    </span>
                </div>
            )}
        </div>
    )

    // Rendered View
    return (
        <PageContainer
            title="So Arm101 Control"
            description="Robot arm joint control and visualization"
            actions={
                <div className="flex items-center gap-4">
                    {/* MODE TOGGLE */}
                    <div className="flex bg-gray-800 rounded-lg p-1 border border-gray-700">
                        <button
                            onClick={() => handleModeSwitch('ROS')}
                            className={`flex items-center gap-2 px-3 py-1.5 text-xs font-medium rounded-md transition-all ${connectionMode === 'ROS' ? 'bg-gray-700 text-white shadow-sm' : 'text-gray-400 hover:text-white'}`}
                        >
                            <Wifi className="w-3 h-3" />
                            ROS Bridge
                        </button>
                        <button
                            onClick={() => handleModeSwitch('DIRECT')}
                            disabled={!isSerialSupported}
                            title={!isSerialSupported ? "Not supported in this browser" : ""}
                            className={`flex items-center gap-2 px-3 py-1.5 text-xs font-medium rounded-md transition-all ${connectionMode === 'DIRECT' ? 'bg-blue-600 text-white shadow-sm' : 'text-gray-400 hover:text-white disabled:opacity-50'}`}
                        >
                            <Usb className="w-3 h-3" />
                            Direct USB
                        </button>
                    </div>

                    {/* CONNECTION BUTTON */}
                    {connectionMode === 'DIRECT' ? (
                        <button
                            onClick={handleUsbConnect}
                            disabled={!isSerialSupported}
                            className={`flex items-center gap-2 px-3 py-1.5 text-xs font-semibold text-white rounded-lg transition-all ${directStatus.connected ? 'bg-red-600 hover:bg-red-700' : 'bg-green-600 hover:bg-green-700'}`}
                        >
                            <Usb className="w-4 h-4" />
                            {directStatus.connected ? 'Disconnect USB' : 'Connect USB'}
                        </button>
                    ) : (
                        connectionIndicator
                    )}

                    <div className="h-6 w-px bg-gray-700 mx-1"></div>

                    <button
                        onClick={handleHomePosition}
                        disabled={!isControlEnabled}
                        className="flex items-center gap-2 px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95"
                    >
                        <Home className="w-4 h-4" />
                        Home
                    </button>

                    {/* KICKSTART TEST BUTTON - Bypasses all React logic */}
                    {connectionMode === 'DIRECT' && directStatus.connected && (
                        <button
                            onClick={handleSerialKickstart}
                            className="flex items-center gap-2 px-3 py-2 bg-orange-600 hover:bg-orange-700 text-white rounded-lg transition-all duration-200 hover:scale-105 active:scale-95 animate-pulse"
                            title="Debug: Send hardcoded 90,90,90,90,90,90 directly to serial"
                        >
                            ⚡ KICKSTART
                        </button>
                    )}
                </div>
            }
        >
            <div className="grid grid-cols-1 xl:grid-cols-4 gap-6 h-[calc(100vh-200px)]" ref={containerRef}>
                {/* Left Panel - Arm Controls */}
                <div className="xl:col-span-1 space-y-6">
                    {/* Arm Info Card */}
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4">
                        <h3 className="text-white font-semibold mb-3 flex items-center gap-2">
                            <Grip className="w-4 h-4 text-cyan-400" />
                            So Arm101
                        </h3>
                        <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                                <span className="text-gray-400">Type:</span>
                                <span className="text-white">6-DOF Robot Arm</span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-400">Mode:</span>
                                <span className="text-blue-400 font-mono text-xs bg-blue-900/30 px-2 py-0.5 rounded">
                                    {connectionMode === 'DIRECT' ? 'WEB SERIAL' : 'ROS BRIDGE'}
                                </span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-400">Status:</span>
                                <span className={isControlEnabled ? 'text-green-400' : 'text-red-400'}>
                                    {isControlEnabled ? 'Connected' : 'Disconnected'}
                                </span>
                            </div>
                        </div>
                    </div>

                    {/* Control Mode Toggle (JOINTS / CARTESIAN) */}
                    <div className="bg-gray-900 border border-gray-800 rounded-xl overflow-hidden">
                        <div className="flex border-b border-gray-800">
                            <button
                                type="button"
                                onClick={() => setControlMode('JOINTS')}
                                className={`flex-1 flex items-center justify-center gap-2 p-3 text-sm font-medium transition-colors
                                    ${controlMode === 'JOINTS' ? 'bg-blue-600 text-white' : 'bg-gray-800 text-gray-300 hover:bg-gray-700'}`}
                            >
                                <Grip className="w-4 h-4" />
                                Joints
                            </button>
                            <button
                                type="button"
                                onClick={() => setControlMode('CARTESIAN')}
                                className={`flex-1 flex items-center justify-center gap-2 p-3 text-sm font-medium transition-colors
                                    ${controlMode === 'CARTESIAN' ? 'bg-blue-600 text-white' : 'bg-gray-800 text-gray-300 hover:bg-gray-700'}`}
                            >
                                <Activity className="w-4 h-4" />
                                Cartesian
                            </button>
                        </div>
                        <div className="p-4">
                            {controlMode === 'JOINTS' && (
                                <>
                                    <div className="flex items-center justify-between mb-3">
                                        <span className="text-gray-400 text-xs font-medium">Joint Control Mode</span>
                                        <button
                                            type="button"
                                            onClick={() =>
                                                setJointControlMode((prev) => (prev === 'basic' ? 'advanced' : 'basic'))
                                            }
                                            className="text-xs text-gray-500 hover:text-primary-400 transition-colors flex items-center gap-1"
                                        >
                                            {jointControlMode === 'basic' ? 'Advanced' : 'Basic'}
                                            <Settings className="w-3 h-3" />
                                        </button>
                                    </div>
                                    <ArmControls
                                        enabled={isControlEnabled}
                                        joints={joints}
                                        onJointChange={handleJointUpdate}
                                    />
                                </>
                            )}

                            {controlMode === 'CARTESIAN' && (
                                <div className="space-y-4">
                                    <div className="grid grid-cols-3 gap-3">
                                        <div>
                                            <label className="block text-xs text-gray-400 mb-1">X-Axis (m)</label>
                                            <input type="number" value={cartesianTarget.x} onChange={(e) => handleCartesianInputChange('x', e.target.value)} className="w-full bg-gray-900 text-white text-xs rounded-lg px-2 py-1.5 border border-gray-700 focus:outline-none focus:ring-2 focus:ring-blue-600" step="0.01" />
                                        </div>
                                        <div>
                                            <label className="block text-xs text-gray-400 mb-1">Y-Axis (m)</label>
                                            <input type="number" value={cartesianTarget.y} onChange={(e) => handleCartesianInputChange('y', e.target.value)} className="w-full bg-gray-900 text-white text-xs rounded-lg px-2 py-1.5 border border-gray-700 focus:outline-none focus:ring-2 focus:ring-blue-600" step="0.01" />
                                        </div>
                                        <div>
                                            <label className="block text-xs text-gray-400 mb-1">Z-Axis (m)</label>
                                            <input type="number" value={cartesianTarget.z} onChange={(e) => handleCartesianInputChange('z', e.target.value)} className="w-full bg-gray-900 text-white text-xs rounded-lg px-2 py-1.5 border border-gray-700 focus:outline-none focus:ring-2 focus:ring-blue-600" step="0.01" />
                                        </div>
                                    </div>
                                    <button
                                        type="button"
                                        onClick={handleMoveToPosition}
                                        disabled={!isControlEnabled || connectionMode === 'DIRECT'}
                                        title={connectionMode === 'DIRECT' ? 'IK not available in Direct Mode' : ''}
                                        className="w-full px-3 py-2 text-xs font-semibold rounded-lg bg-green-600 hover:bg-green-700 text-white transition-colors disabled:bg-gray-700 disabled:cursor-not-allowed"
                                    >
                                        Move to Position
                                    </button>
                                </div>
                            )}
                        </div>
                    </div>
                </div>

                {/* Center Panel - Visualization */}
                <div className="xl:col-span-2">
                    <div className="bg-gray-900 border border-gray-800 rounded-xl h-full relative overflow-hidden flex flex-col">
                        <div className="absolute top-4 right-4 z-10 flex gap-2">
                            {/* Optional overlay buttons */}
                            <button onClick={toggleFullscreen} className="p-2 bg-gray-800/80 hover:bg-gray-700/80 text-white rounded-lg transition-colors">
                                <Maximize2 className="w-4 h-4" />
                            </button>
                        </div>

                        {/* CONDITIONAL RENDER: 3D SCENE vs RVIZ */}
                        <div className="flex-1 w-full h-full">
                            {connectionMode === 'DIRECT' ? (
                                <React.Suspense fallback={
                                    <div className="w-full h-full flex items-center justify-center bg-black text-gray-500 font-mono text-xs">
                                        <div className="flex flex-col items-center gap-3">
                                            <Loader className="w-8 h-8 animate-spin text-cyan-400" />
                                            <span>Loading 3D Scene...</span>
                                        </div>
                                    </div>
                                }>
                                    <LiveRobotScene joints={joints} />
                                </React.Suspense>
                            ) : (
                                <RvizPanel />
                            )}
                        </div>
                    </div>
                </div>

                {/* Right Panel - Status */}
                <div className="xl:col-span-1 space-y-6">
                    <StatusPanel
                        status={isControlEnabled ? 'RUNNING' : 'STOPPED'}
                        telemetryData={telemetryData}
                        isRunning={isControlEnabled}
                    />

                    {/* End Effector Position */}
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4">
                        <h3 className="text-white font-semibold mb-4 flex items-center gap-2">
                            <Activity className="w-4 h-4 text-purple-400" />
                            End Effector
                        </h3>
                        <div className="bg-gray-800/30 rounded-lg p-3">
                            <div className="grid grid-cols-3 gap-2 text-xs">
                                <div><span className="text-gray-500">X:</span><span className="ml-1 text-white font-mono">0.00</span></div>
                                <div><span className="text-gray-500">Y:</span><span className="ml-1 text-white font-mono">0.00</span></div>
                                <div><span className="text-gray-500">Z:</span><span className="ml-1 text-white font-mono">0.00</span></div>
                            </div>
                        </div>
                        <p className="text-gray-500 text-xs mt-3 text-center">Position data from /so_arm101/end_effector</p>
                    </div>
                </div>
            </div>
        </PageContainer>
    )
}

const SoArm101 = () => {
    const handleRetry = () => {
        window.location.reload()
    }

    return (
        <ErrorBoundary onRetry={handleRetry}>
            <SoArm101Content />
        </ErrorBoundary>
    )
}

export default SoArm101
