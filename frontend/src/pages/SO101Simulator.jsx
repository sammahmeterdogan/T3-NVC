// frontend/src/pages/SO101Simulator.jsx
// SO-ARM101 Dedicated Simulator - Isolated from Turtlebot
// Supports both ROS Simulation and Physical Robot via WebSerial
import { useState, useEffect, useRef, useCallback } from 'react'
import toast from 'react-hot-toast'
import {
    Play, Square, Maximize2, Minimize2, RotateCcw,
    Hand, GripHorizontal, AlertTriangle, Wifi, WifiOff,
    Loader2, Target, Zap, RefreshCw, Usb, Unplug
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import ErrorBoundary from '../components/ui/ErrorBoundary'
import { createSoarmRosClient } from '../services/soarmRosClient'
import { webSerialService } from '../services/webSerial'

// NoVNC URL for SO-ARM101 (port 6083 mapped from Docker's 6081)
const NOVNC_URL = 'http://localhost:6081/vnc.html?resize=scale&autoconnect=1'

// Servo ID mapping for SO-ARM101 (6 DOF + Gripper)
const SERVO_IDS = {
    BASE: 1,
    SHOULDER: 2,
    ELBOW: 3,
    WRIST_PITCH: 4,
    WRIST_ROLL: 5,
    GRIPPER: 6
}

// Convert radians to servo steps (0-4095 range, center at 2048)
function radiansToSteps(radians, minRad = -Math.PI, maxRad = Math.PI) {
    const normalized = (radians - minRad) / (maxRad - minRad)
    return Math.floor(normalized * 4095)
}

const SO101SimulatorContent = () => {
    // Connection state
    const [rosState, setRosState] = useState('DISCONNECTED')
    const [isReconnecting, setIsReconnecting] = useState(false)
    const [serialConnected, setSerialConnected] = useState(false)

    // Motion planning state
    const [isPlanning, setIsPlanning] = useState(false)
    const [isExecuting, setIsExecuting] = useState(false)
    const [pathReady, setPathReady] = useState(false)
    const [plannedTrajectory, setPlannedTrajectory] = useState(null)

    // UI state
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [iframeKey, setIframeKey] = useState(0)
    const [executionMode, setExecutionMode] = useState('simulation') // 'simulation' | 'physical' | 'both'

    // Refs
    const containerRef = useRef(null)
    const rosClientRef = useRef(null)
    const mountedRef = useRef(true)

    // Initialize isolated ROS client
    useEffect(() => {
        mountedRef.current = true
        const client = createSoarmRosClient()
        rosClientRef.current = client

        // Listen to state changes
        const unsubscribe = client.onStateChange((state) => {
            if (!mountedRef.current) return
            setRosState(state)
            setIsReconnecting(state === 'CONNECTING')

            if (state === 'CONNECTED') {
                toast.success('SO-ARM101 ROS Connected', { id: 'ros-status' })
            } else if (state === 'DISCONNECTED') {
                toast.error('SO-ARM101 ROS Disconnected', { id: 'ros-status' })
            }
        })

        // Connect
        client.connect().catch((err) => {
            console.error('[SO101Simulator] Initial connection failed:', err)
        })

        return () => {
            mountedRef.current = false
            unsubscribe()
            client.destroy()
            rosClientRef.current = null
        }
    }, [])

    // Check if connected
    const isConnected = rosState === 'CONNECTED'

    // WebSerial connection handler
    const handleSerialConnect = useCallback(async () => {
        if (serialConnected) {
            await webSerialService.disconnect()
            setSerialConnected(false)
            toast.success('USB disconnected')
        } else {
            try {
                await webSerialService.connect()
                setSerialConnected(true)
                toast.success('USB connected to SO-ARM101')
            } catch (error) {
                toast.error(`USB connection failed: ${error.message}`)
            }
        }
    }, [serialConnected])

    // Fullscreen toggle
    const toggleFullscreen = useCallback(() => {
        if (!containerRef.current) return

        if (!document.fullscreenElement) {
            containerRef.current.requestFullscreen?.()
            setIsFullscreen(true)
        } else {
            document.exitFullscreen?.()
            setIsFullscreen(false)
        }
    }, [])

    // Refresh iframe
    const refreshViewport = useCallback(() => {
        setIframeKey(prev => prev + 1)
        toast.success('Viewport refreshed')
    }, [])

    // Motion planning commands (via ROS)
    const handlePlan = useCallback(async () => {
        if (!isConnected) return toast.error('ROS not connected')

        setIsPlanning(true)
        setPathReady(false)
        setPlannedTrajectory(null)
        try {
            // Send plan command to MoveIt wrapper
            rosClientRef.current?.sendPlanCmd()

            // TODO: Listen for /display_planned_path to get trajectory
            // For now, assume success after a short delay to allow "Execute"
            setTimeout(() => {
                setPathReady(true)
                // Mock trajectory for physical test if real one missing
                setPlannedTrajectory([{ base: 0.5, shoulder: 0.5, time_ms: 2000 }])
                toast.success('Motion path plan request sent')
            }, 1000)

        } catch (error) {
            toast.error(`Planning failed: ${error.message}`)
        } finally {
            setIsPlanning(false)
        }
    }, [isConnected])

    // Execute trajectory on physical robot via WebSerial
    const executePhysicalTrajectory = useCallback(async (trajectory) => {
        if (!serialConnected || !trajectory) {
            console.warn('[SO101] No serial or trajectory for physical execution')
            return
        }

        console.log('[SO101] Executing physical trajectory:', trajectory)

        // Enable torque on all servos
        await webSerialService.torqueEnable(true)

        // Iterate through trajectory points
        for (const point of trajectory) {
            if (!mountedRef.current) break // Stop if unmounted

            const servos = [
                { id: SERVO_IDS.BASE, position: radiansToSteps(point.base || 0) },
                { id: SERVO_IDS.SHOULDER, position: radiansToSteps(point.shoulder || 0) },
                { id: SERVO_IDS.ELBOW, position: radiansToSteps(point.elbow || 0) },
                { id: SERVO_IDS.WRIST_PITCH, position: radiansToSteps(point.wrist_pitch || 0) },
                { id: SERVO_IDS.WRIST_ROLL, position: radiansToSteps(point.wrist_roll || 0) },
            ]

            // Sync write to all servos simultaneously
            await webSerialService.syncWritePositions(servos, point.time_ms || 100)

            // Wait for movement (add delay between trajectory points)
            await new Promise(resolve => setTimeout(resolve, point.time_ms || 50))
        }

        console.log('[SO101] Physical trajectory execution complete')
    }, [serialConnected])

    // Unified execute handler
    const handleExecute = useCallback(async () => {
        if (!isConnected && executionMode !== 'physical') {
            return toast.error('ROS not connected')
        }

        setIsExecuting(true)
        try {
            // Execute in simulation (ROS/MoveIt)
            if (executionMode === 'simulation' || executionMode === 'both') {
                rosClientRef.current?.sendExecuteCmd()
            }

            // Execute on physical robot
            if ((executionMode === 'physical' || executionMode === 'both') && serialConnected) {
                if (plannedTrajectory) {
                    await executePhysicalTrajectory(plannedTrajectory)
                } else {
                    toast.warning('No trajectory for physical execution')
                }
            }

            toast.success('Execution command sent')
            setPathReady(false)
        } catch (error) {
            toast.error(`Execution failed: ${error.message}`)
        } finally {
            setIsExecuting(false)
        }
    }, [isConnected, executionMode, serialConnected, plannedTrajectory, executePhysicalTrajectory])

    const handleStop = useCallback(async () => {
        try {
            // Send via ROS
            rosClientRef.current?.sendStopCmd()

            // Disable physical robot torque (free move)
            if (serialConnected) {
                await webSerialService.torqueEnable(false)
            }

            toast.success('Motion stopped')
            setIsPlanning(false)
            setIsExecuting(false)
        } catch (error) {
            toast.error(`Stop failed: ${error.message}`)
        }
    }, [serialConnected])

    const handleHome = useCallback(async () => {
        if (!isConnected && !serialConnected) return toast.error('No connection available')

        try {
            // ROS home command
            if (isConnected) {
                rosClientRef.current?.sendHomeCmd()
            }

            // Physical robot home (center all servos)
            if (serialConnected) {
                const homeServos = Object.values(SERVO_IDS).slice(0, 5).map(id => ({
                    id,
                    position: 2048 // Center position
                }))
                await webSerialService.syncWritePositions(homeServos, 1000)
            }

            toast.success('Moving to home position...')
        } catch (error) {
            toast.error(`Home failed: ${error.message}`)
        }
    }, [isConnected, serialConnected])

    const handleGripper = useCallback(async (open) => {
        if (!isConnected && !serialConnected) return toast.error('No connection available')

        try {
            // ROS gripper command
            if (isConnected) {
                rosClientRef.current?.sendGripperCmd(open)
            }

            // Physical gripper
            if (serialConnected) {
                const position = open ? 3500 : 500 // Open = wide, Close = narrow
                await webSerialService.setPosition(SERVO_IDS.GRIPPER, position, 500)
            }

            toast.success(`Gripper ${open ? 'opened' : 'closed'}`)
        } catch (error) {
            toast.error(`Gripper failed: ${error.message}`)
        }
    }, [isConnected, serialConnected])

    // Emergency Stop
    const handleEmergencyStop = useCallback(async () => {
        console.log('[SO101Simulator] EMERGENCY STOP ACTIVATED')

        // ROS E-Stop
        rosClientRef.current?.sendEstop()

        // Physical E-Stop - disable all torque immediately
        if (serialConnected) {
            try {
                await webSerialService.torqueEnable(false)
            } catch { /* ignore */ }
        }

        setIsPlanning(false)
        setIsExecuting(false)
        setPathReady(false)
        toast.error('🚨 EMERGENCY STOP ACTIVATED', { duration: 5000 })
    }, [serialConnected])

    // Status badge component
    const StatusBadge = () => (
        <div className="flex items-center gap-3">
            {/* ROS Status */}
            <div className={`flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-semibold transition-all ${isConnected
                ? 'bg-green-500/20 text-green-400 border border-green-500/30'
                : isReconnecting
                    ? 'bg-yellow-500/20 text-yellow-400 border border-yellow-500/30'
                    : 'bg-red-500/20 text-red-400 border border-red-500/30'
                }`}>
                {isConnected ? (
                    <><Wifi className="w-3 h-3" /> ROS</>
                ) : isReconnecting ? (
                    <><Loader2 className="w-3 h-3 animate-spin" /> ROS</>
                ) : (
                    <><WifiOff className="w-3 h-3" /> ROS</>
                )}
            </div>

            {/* USB Status */}
            <button
                onClick={handleSerialConnect}
                className={`flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-semibold transition-all cursor-pointer ${serialConnected
                    ? 'bg-blue-500/20 text-blue-400 border border-blue-500/30 hover:bg-blue-500/30'
                    : 'bg-gray-500/20 text-gray-400 border border-gray-500/30 hover:bg-gray-500/30'
                    }`}
            >
                {serialConnected ? (
                    <><Usb className="w-3 h-3" /> USB</>
                ) : (
                    <><Unplug className="w-3 h-3" /> USB</>
                )}
            </button>
        </div>
    )

    return (
        <PageContainer
            title="SO-ARM101 Simulator"
            description="MoveIt 2 Motion Planning & Physical Control"
            actions={<StatusBadge />}
        >
            <div
                ref={containerRef}
                className="grid grid-cols-1 xl:grid-cols-4 gap-4 h-[calc(100vh-180px)]"
            >
                {/* ============ MAIN VIEWPORT (NoVNC) ============ */}
                <div className="xl:col-span-3 relative">
                    <div className="bg-gray-900 border border-gray-800 rounded-xl h-full overflow-hidden relative">
                        {/* Viewport Controls */}
                        <div className="absolute top-3 right-3 z-20 flex gap-2">
                            <button
                                onClick={refreshViewport}
                                className="p-2 bg-gray-800/90 hover:bg-gray-700 text-gray-300 rounded-lg transition-all backdrop-blur-sm"
                                title="Refresh Viewport"
                            >
                                <RefreshCw className="w-4 h-4" />
                            </button>
                            <button
                                onClick={toggleFullscreen}
                                className="p-2 bg-gray-800/90 hover:bg-gray-700 text-gray-300 rounded-lg transition-all backdrop-blur-sm"
                                title={isFullscreen ? 'Exit Fullscreen' : 'Fullscreen'}
                            >
                                {isFullscreen ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
                            </button>
                        </div>

                        {/* Reconnecting Overlay */}
                        {!isConnected && (
                            <div className="absolute inset-0 z-10 bg-gray-900/80 backdrop-blur-sm flex flex-col items-center justify-center gap-4">
                                <Loader2 className="w-12 h-12 text-blue-500 animate-spin" />
                                <p className="text-gray-300 font-medium">
                                    {isReconnecting ? 'Reconnecting to SO-ARM101...' : 'Waiting for ROS connection...'}
                                </p>
                                <p className="text-gray-500 text-sm">
                                    Target: /rosbridge-soarm → 127.0.0.1:9092
                                </p>
                            </div>
                        )}

                        {/* NoVNC Iframe */}
                        <iframe
                            key={iframeKey}
                            src={NOVNC_URL}
                            className="w-full h-full border-0"
                            title="SO-ARM101 RViz Viewport"
                            allow="clipboard-read; clipboard-write"
                        />
                    </div>
                </div>

                {/* ============ RIGHT PANEL (Controls) ============ */}
                <div className="xl:col-span-1 flex flex-col gap-4">
                    {/* Status Card */}
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4">
                        <h3 className="text-white font-semibold mb-3 flex items-center gap-2">
                            <Target className="w-4 h-4 text-blue-400" />
                            Robot Status
                        </h3>
                        <div className="space-y-2 text-sm">
                            <div className="flex justify-between">
                                <span className="text-gray-400">Robot</span>
                                <span className="text-white font-mono">SO-ARM101</span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-400">ROS Bridge</span>
                                <span className={isConnected ? 'text-green-400' : 'text-red-400'}>
                                    {rosState}
                                </span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-400">USB Serial</span>
                                <span className={serialConnected ? 'text-blue-400' : 'text-gray-500'}>
                                    {serialConnected ? 'Connected' : 'Not Connected'}
                                </span>
                            </div>
                            <div className="flex justify-between">
                                <span className="text-gray-400">Path Status</span>
                                <span className={pathReady ? 'text-green-400' : 'text-gray-500'}>
                                    {pathReady ? 'Ready' : 'No Path'}
                                </span>
                            </div>
                        </div>

                        {/* Execution Mode Selector */}
                        <div className="mt-3 pt-3 border-t border-gray-800">
                            <span className="text-[10px] text-gray-500 uppercase tracking-wider font-bold block mb-2">
                                Execution Mode
                            </span>
                            <div className="grid grid-cols-3 gap-1 bg-gray-800 p-1 rounded-lg">
                                {['simulation', 'physical', 'both'].map(mode => (
                                    <button
                                        key={mode}
                                        onClick={() => setExecutionMode(mode)}
                                        className={`px-2 py-1 text-[10px] font-semibold rounded transition-all capitalize ${executionMode === mode
                                            ? 'bg-blue-600 text-white'
                                            : 'text-gray-400 hover:text-white'
                                            }`}
                                    >
                                        {mode}
                                    </button>
                                ))}
                            </div>
                        </div>
                    </div>

                    {/* Motion Controls */}
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4 flex-1">
                        <h3 className="text-white font-semibold mb-4 flex items-center gap-2">
                            <Zap className="w-4 h-4 text-yellow-400" />
                            Motion Control
                        </h3>

                        <div className="space-y-3">
                            {/* Plan & Execute */}
                            <div className="grid grid-cols-2 gap-2">
                                <button
                                    onClick={handlePlan}
                                    disabled={!isConnected || isPlanning}
                                    className="flex flex-col items-center justify-center gap-2 p-4 bg-blue-900/30 hover:bg-blue-900/50 text-blue-400 border border-blue-800/50 rounded-xl transition-all disabled:opacity-40 disabled:cursor-not-allowed"
                                >
                                    {isPlanning ? (
                                        <Loader2 className="w-6 h-6 animate-spin" />
                                    ) : (
                                        <Target className="w-6 h-6" />
                                    )}
                                    <span className="text-xs font-bold">PLAN</span>
                                </button>

                                <button
                                    onClick={handleExecute}
                                    disabled={(!isConnected && !serialConnected) || !pathReady || isExecuting}
                                    className="flex flex-col items-center justify-center gap-2 p-4 bg-green-900/30 hover:bg-green-900/50 text-green-400 border border-green-800/50 rounded-xl transition-all disabled:opacity-40 disabled:cursor-not-allowed"
                                >
                                    {isExecuting ? (
                                        <Loader2 className="w-6 h-6 animate-spin" />
                                    ) : (
                                        <Play className="w-6 h-6" />
                                    )}
                                    <span className="text-xs font-bold">EXECUTE</span>
                                </button>
                            </div>

                            {/* Stop & Home */}
                            <div className="grid grid-cols-2 gap-2">
                                <button
                                    onClick={handleStop}
                                    className="flex items-center justify-center gap-2 p-3 bg-orange-900/30 hover:bg-orange-900/50 text-orange-400 border border-orange-800/50 rounded-xl transition-all"
                                >
                                    <Square className="w-4 h-4 fill-current" />
                                    <span className="text-xs font-bold">STOP</span>
                                </button>

                                <button
                                    onClick={handleHome}
                                    disabled={!isConnected && !serialConnected}
                                    className="flex items-center justify-center gap-2 p-3 bg-gray-800 hover:bg-gray-700 text-gray-300 border border-gray-700 rounded-xl transition-all disabled:opacity-40"
                                >
                                    <RotateCcw className="w-4 h-4" />
                                    <span className="text-xs font-bold">HOME</span>
                                </button>
                            </div>

                            {/* Gripper Controls */}
                            <div className="pt-3 border-t border-gray-800">
                                <span className="text-[10px] text-gray-500 uppercase tracking-wider font-bold block mb-2">
                                    End Effector
                                </span>
                                <div className="grid grid-cols-2 gap-2">
                                    <button
                                        onClick={() => handleGripper(true)}
                                        disabled={!isConnected && !serialConnected}
                                        className="flex items-center justify-center gap-2 p-2.5 bg-gray-800 hover:bg-gray-700 text-gray-300 text-xs border border-gray-700 rounded-lg transition-all disabled:opacity-40"
                                    >
                                        <Hand className="w-3.5 h-3.5" /> Open
                                    </button>
                                    <button
                                        onClick={() => handleGripper(false)}
                                        disabled={!isConnected && !serialConnected}
                                        className="flex items-center justify-center gap-2 p-2.5 bg-gray-800 hover:bg-gray-700 text-gray-300 text-xs border border-gray-700 rounded-lg transition-all disabled:opacity-40"
                                    >
                                        <GripHorizontal className="w-3.5 h-3.5" /> Close
                                    </button>
                                </div>
                            </div>
                        </div>
                    </div>

                    {/* EMERGENCY STOP */}
                    <button
                        onClick={handleEmergencyStop}
                        className="flex items-center justify-center gap-3 p-4 bg-red-600 hover:bg-red-500 active:bg-red-700 text-white font-bold text-lg rounded-xl transition-all shadow-lg shadow-red-900/50 border-2 border-red-500"
                    >
                        <AlertTriangle className="w-6 h-6" />
                        EMERGENCY STOP
                    </button>
                </div>
            </div>
        </PageContainer>
    )
}

const SO101Simulator = () => (
    <ErrorBoundary>
        <SO101SimulatorContent />
    </ErrorBoundary>
)

export default SO101Simulator
