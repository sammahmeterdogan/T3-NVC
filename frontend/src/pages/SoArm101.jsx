
import React, { useState, useEffect, useRef, useCallback } from 'react'
import toast from 'react-hot-toast'
import {
    Settings, Maximize2, Activity,
    Home, Grip, Usb, Wifi, Loader, Target, Sliders,
    Power, Circle, Play, Trash2, List, Pause, Square
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
import solveIK from '../utils/inverseKinematics'
import computeFK from '../utils/forwardKinematics'
import { isInWorkspace } from '../utils/robotConfig'

const SoArm101Content = () => {
    const [jointControlMode, setJointControlMode] = useState(() => {
        return localStorage.getItem('soarm101_joint_control_mode') || 'FK' // 'FK' | 'IK'
    })
    const [controlMode, setControlMode] = useState(() => {
        return localStorage.getItem('soarm101_mode') || 'JOINTS' // JOINTS | CARTESIAN
    })

    // --- IK MODE STATE ---
    const [ikTarget, setIKTarget] = useState([0.15, 0.0, 0.10]) // Initial reachable position
    const [ikReachable, setIKReachable] = useState(true)

    // --- SEQUENCE PROGRAMMER STATE ---
    const [sequenceSteps, setSequenceSteps] = useState([])
    const [isPlaying, setIsPlaying] = useState(false)
    const [isTorqueEnabled, setIsTorqueEnabled] = useState(true)

    // Recording Refs (for high-frequency capture without re-rendering)
    const [isRecording, setIsRecording] = useState(false)
    const isRecordingRef = useRef(false)
    const recordingStartTimeRef = useRef(0)
    const recordingFramesRef = useRef([])

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
    // Track the last successfully sent IK command for display
    const [lastCommandedPose, setLastCommandedPose] = useState(null)

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

        // --- MOTION RECORDING HOOK ---
        if (isRecordingRef.current) {
            const t = Date.now() - recordingStartTimeRef.current
            recordingFramesRef.current.push({
                t: t,
                joints: { ...newJoints }
            })
        }

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
            setLastCommandedPose({ x, y, z }) // Update display with successful command
            toast.success('Move Executed')
        } catch (e) {
            toast.error('IK move failed: ' + (e.message || 'Unknown error'))
        }
    }

    // --- IK MODE HANDLERS ---

    /**
     * Handle IK target drag - real-time IK solving
     */
    const handleIKTargetDrag = useCallback((newPosition) => {
        // Update target position
        setIKTarget(newPosition)

        // Check if position is reachable
        const reachable = isInWorkspace(newPosition)
        setIKReachable(reachable)
        console.log(`[IK] IK_REACHABLE=${reachable}`)

        if (!reachable) {
            console.warn('[IK] IK_REACHABLE=false reason=OUT_OF_WORKSPACE')
            return
        }

        // Solve IK using JS solver (Approach 2: solve locally, publish JOINTS)
        const solution = solveIK(newPosition, {
            wristRoll: joints.wristRoll,
            gripper: joints.gripper
        })

        if (!solution) {
            console.warn('[IK] IK_SOLVE ok=false reason=NO_IK_SOLUTION')
            setIKReachable(false)
            return
        }

        console.log('[IK] IK_SOLVE ok=true')
        console.log(`[IK] IK_JOINTS base=${solution.base.toFixed(1)} shoulder=${solution.shoulder.toFixed(1)} elbow=${solution.elbow.toFixed(1)} wristPitch=${solution.wristPitch.toFixed(1)}`)

        // Update joints state (triggers visual update)
        setJoints(solution)

        // Update commanded pose display
        setLastCommandedPose({ x: newPosition[0], y: newPosition[1], z: newPosition[2] })

        // Throttled servo command (same as slider logic)
        const now = Date.now()
        if (now - lastNetworkUpdateRef.current < NETWORK_THROTTLE_MS) {
            console.log('[IK] THROTTLED - skipping command send')
            return
        }
        lastNetworkUpdateRef.current = now

        // Send to hardware via SAME pipeline as sliders
        console.log(`[IK] DISPATCH_JOINTS mode=${connectionMode} directConnected=${directStatus.connected} rosConnected=${rosClient.isConnected()}`)

        if (connectionMode === 'DIRECT' && directStatus.connected) {
            const servoCommands = [
                { id: 1, position: webSerialService.degreesToSteps(solution.base, -180, 180) },
                { id: 2, position: webSerialService.degreesToSteps(solution.shoulder, -90, 90) },
                { id: 3, position: webSerialService.degreesToSteps(solution.elbow, -135, 135) },
                { id: 4, position: webSerialService.degreesToSteps(solution.wristPitch, -90, 90) },
                { id: 5, position: webSerialService.degreesToSteps(solution.wristRoll, -180, 180) },
                { id: 6, position: Math.floor((solution.gripper / 100) * SCS.MAX_POSITION) }
            ]

            console.log('[IK] IK_COMMAND_SENT transport=DIRECT_USB payload=', servoCommands.map(s => `ID${s.id}:${s.position}`).join(', '))
            webSerialService.syncWritePositions(servoCommands, 200).catch(err => {
                console.error('[IK] SERIAL_WRITE_ERROR', err)
                if (err.name === 'NetworkError') {
                    setDirectStatus({ connected: false, portLabel: '' })
                    toast.error('Device disconnected')
                }
            })
        } else if (connectionMode === 'ROS' && rosClient.isConnected()) {
            console.log('[IK] IK_COMMAND_SENT transport=ROS_BRIDGE payload=', solution)
            armAPI.sendJoints(solution).catch((err) => {
                console.error('[IK] ROS_PUBLISH_ERROR', err)
                toast.error('Failed to send IK command')
            })
        } else {
            // NO TRANSPORT AVAILABLE - this is why hardware doesn't move!
            console.warn(`[IK] NO_TRANSPORT_AVAILABLE mode=${connectionMode} directConnected=${directStatus.connected} rosConnected=${rosClient.isConnected()}`)
            console.warn('[IK] Hardware will NOT move. Please connect to the robot first.')
        }
    }, [joints.wristRoll, joints.gripper, connectionMode, directStatus.connected])

    /**
     * SEQUENCE PROGRAMMER HANDLERS
     * Implements Torque Control, Recording, and Playback
     */

    const handleTorqueToggle = useCallback(() => {
        const newTorqueState = !isTorqueEnabled
        setIsTorqueEnabled(newTorqueState)
        console.log(`[SEQUENCE] TORQUE_UI_SET enabled=${newTorqueState} mode=${connectionMode}`)

        if (connectionMode === 'DIRECT' && directStatus.connected) {
            // Direct USB Torque Control (Broadcast 0x28)
            webSerialService.torqueEnable(newTorqueState).catch(err => {
                toast.error('Failed to toggle torque')
                setIsTorqueEnabled(!newTorqueState) // Revert on fail
            })
        } else if (connectionMode === 'ROS' && rosClient.isConnected()) {
            // ROS Torque Control
            armAPI.setTorque(newTorqueState).catch(err => {
                toast.error('Failed to toggle torque')
                setIsTorqueEnabled(!newTorqueState)
            })
        } else {
            console.warn(`[SEQUENCE] TORQUE_CMD_SKIPPED mode=${connectionMode} connected=false`)
        }
    }, [isTorqueEnabled, connectionMode, directStatus.connected])

    const handleRecordToggle = useCallback(() => {
        if (isRecording) {
            // STOP RECORDING
            isRecordingRef.current = false
            setIsRecording(false)

            const frames = recordingFramesRef.current
            const duration = Date.now() - recordingStartTimeRef.current

            if (frames.length === 0) {
                toast('No motion recorded')
                return
            }

            // Normalize frames (start at t=0)
            const startTime = frames[0].t
            const normalizedFrames = frames.map(f => ({
                t: f.t - startTime,
                joints: f.joints
            }))

            const newSequence = {
                id: Date.now(),
                timestamp: new Date().toISOString(),
                duration: duration,
                frames: normalizedFrames,
                frameCount: normalizedFrames.length,
                // Store a snapshot of first frame for list display ("B:10 S:-20...")
                snapshot: normalizedFrames[0].joints
            }

            setSequenceSteps(prev => [...prev, newSequence])
            console.log(`[SEQUENCE] RECORD_STOP frames=${frames.length} duration=${duration}ms`)
            toast.success(`Recording Saved (${(duration / 1000).toFixed(1)}s)`)

        } else {
            // START RECORDING
            recordingFramesRef.current = []
            // Capture initial pose as frame 0
            recordingFramesRef.current.push({ t: 0, joints: { ...joints } })

            recordingStartTimeRef.current = Date.now()
            isRecordingRef.current = true
            setIsRecording(true)

            console.log('[SEQUENCE] RECORD_START')
            toast('Recording Started... Move the robot!')
        }
    }, [isRecording, joints])

    const handleDeleteStep = useCallback((id) => {
        setSequenceSteps(prev => prev.filter(step => step.id !== id))
    }, [])

    const handleClearSequence = useCallback(() => {
        setSequenceSteps([])
        console.log('[SEQUENCE] STEPS_CLEARED')
        toast.success('Sequence Cleared')
    }, [])

    const handlePlaySequence = useCallback(async () => {
        if (sequenceSteps.length === 0) return
        if (isPlaying) {
            setIsPlaying(false)
            return
        }

        // Capture initial torque state
        const wasTorqueEnabled = isTorqueEnabled
        console.log(`[SEQUENCE] PLAY_START prevTorque=${wasTorqueEnabled}`)

        // Force Torque ON if needed
        if (!wasTorqueEnabled) {
            console.log('[SEQUENCE] PLAY_FORCE_TORQUE_ON')
            setIsTorqueEnabled(true)

            if (connectionMode === 'DIRECT' && directStatus.connected) {
                await webSerialService.torqueEnable(true)
            } else if (connectionMode === 'ROS' && rosClient.isConnected()) {
                await armAPI.setTorque(true).catch(console.error)
            }
            // Brief wait for torque
            await new Promise(r => setTimeout(r, 200))
        }

        setIsPlaying(true)
        console.log(`[SEQUENCE] START_PLAYBACK sequences=${sequenceSteps.length}`)

        try {
            // Iterate over ALL recorded sequences
            for (let i = 0; i < sequenceSteps.length; i++) {
                const seq = sequenceSteps[i]
                console.log(`[SEQUENCE] PLAY_SEQUENCE id=${seq.id} duration=${seq.duration}ms frames=${seq.frameCount}`)

                const frames = seq.frames
                if (!frames || frames.length === 0) continue

                const startPlayTime = Date.now()

                // Play frames
                for (let f = 0; f < frames.length; f++) {
                    const frame = frames[f]
                    const targetTime = frame.t

                    // Simple drift-correction sleep
                    let now = Date.now() - startPlayTime
                    if (now < targetTime) {
                        await new Promise(r => setTimeout(r, targetTime - now))
                    }

                    handleJointUpdate(frame.joints)
                }

                console.log(`[SEQUENCE] SEQUENCE_DONE id=${seq.id}`)
                await new Promise(r => setTimeout(r, 500)) // Pause between sequences
            }
            toast.success('Playback Complete')
        } catch (err) {
            console.error('[SEQUENCE] PLAYBACK_ERROR', err)
            toast.error('Playback Failed')
        } finally {
            console.log(`[SEQUENCE] PLAY_DONE restoreTorque=${wasTorqueEnabled}`)

            // Restore Original Torque State
            if (!wasTorqueEnabled) {
                setIsTorqueEnabled(false)
                if (connectionMode === 'DIRECT' && directStatus.connected) {
                    await webSerialService.torqueEnable(false)
                } else if (connectionMode === 'ROS' && rosClient.isConnected()) {
                    await armAPI.setTorque(false).catch(console.error)
                }
            }

            setIsPlaying(false)
            console.log('[SEQUENCE] PLAYBACK_COMPLETE')
        }
    }, [sequenceSteps, isPlaying, handleJointUpdate, isTorqueEnabled, connectionMode, directStatus.connected])

    /**
     * Handle FK/IK mode switch
     */
    const handleJointControlModeSwitch = (mode) => {
        setJointControlMode(mode)

        if (mode === 'IK') {
            // Initialize target at current end-effector position
            const currentEE = computeFK(joints)
            setIKTarget(currentEE)
            setIKReachable(true)
            toast('🎯 IK Mode: Drag the target sphere', { duration: 3000 })
        } else {
            toast('🎛️ FK Mode: Use sliders', { duration: 2000 })
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
                                        <div className="flex bg-gray-800 rounded-lg p-1 border border-gray-700">
                                            <button
                                                type="button"
                                                onClick={() => handleJointControlModeSwitch('FK')}
                                                className={`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors ${jointControlMode === 'FK' ? 'bg-gray-700 text-white' : 'text-gray-400 hover:text-white'}`}
                                            >
                                                <Sliders className="w-3 h-3" />
                                                FK
                                            </button>
                                            <button
                                                type="button"
                                                onClick={() => handleJointControlModeSwitch('IK')}
                                                className={`flex items-center gap-1.5 px-2 py-1 text-xs font-medium rounded transition-colors ${jointControlMode === 'IK' ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'}`}
                                            >
                                                <Target className="w-3 h-3" />
                                                IK
                                            </button>
                                        </div>
                                    </div>
                                    {/* IK Mode Slider Disabled Banner */}
                                    {jointControlMode === 'IK' && (
                                        <div className="bg-yellow-900/30 border border-yellow-700 rounded-lg p-3 mb-3">
                                            <div className="text-yellow-400 text-xs font-medium flex items-center gap-2">
                                                <Target className="w-4 h-4" />
                                                SLIDERS DISABLED - IK MODE ACTIVE
                                            </div>
                                            <div className="text-yellow-600 text-xs mt-1">
                                                Drag the target sphere in the 3D view to control the arm
                                            </div>
                                        </div>
                                    )}
                                    <ArmControls
                                        enabled={isControlEnabled && jointControlMode === 'FK'}
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
                        {/* IK MODE ACTIVE Badge (top-left) */}
                        {jointControlMode === 'IK' && (
                            <div className="absolute top-4 left-4 z-10">
                                <div className="bg-blue-600 text-white px-3 py-1.5 rounded-lg text-xs font-bold flex items-center gap-2 shadow-lg">
                                    <div className="w-2 h-2 bg-white rounded-full animate-pulse" />
                                    IK MODE ACTIVE
                                </div>
                            </div>
                        )}

                        {/* IK TARGET Position Overlay (bottom-left) */}
                        {jointControlMode === 'IK' && (
                            <div className="absolute bottom-4 left-4 z-10 bg-gray-900/95 border border-gray-700 rounded-lg p-3 text-xs font-mono shadow-lg">
                                <div className="text-gray-400 mb-2 font-sans font-medium">Target Position</div>
                                <div className="space-y-1">
                                    <div className="flex justify-between gap-6">
                                        <span className="text-red-400">X:</span>
                                        <span className="text-white">{(ikTarget[0] * 1000).toFixed(1)} mm</span>
                                    </div>
                                    <div className="flex justify-between gap-6">
                                        <span className="text-green-400">Y:</span>
                                        <span className="text-white">{(ikTarget[1] * 1000).toFixed(1)} mm</span>
                                    </div>
                                    <div className="flex justify-between gap-6">
                                        <span className="text-blue-400">Z:</span>
                                        <span className="text-white">{(ikTarget[2] * 1000).toFixed(1)} mm</span>
                                    </div>
                                </div>
                                <div className={`mt-2 pt-2 border-t border-gray-700 font-sans font-bold ${ikReachable ? 'text-green-400' : 'text-red-400'}`}>
                                    {ikReachable ? '✓ REACHABLE' : '✗ OUT OF RANGE'}
                                </div>
                            </div>
                        )}

                        <div className="absolute top-4 right-4 z-10 flex gap-2">
                            {/* IK TARGET Toggle Button */}
                            <button
                                onClick={() => {
                                    const newMode = jointControlMode === 'IK' ? 'FK' : 'IK'
                                    handleJointControlModeSwitch(newMode)
                                    console.log('[IK] IK_MODE=' + (newMode === 'IK'))
                                }}
                                className={`px-3 py-2 rounded-lg text-xs font-bold flex items-center gap-2 shadow-lg transition-colors ${jointControlMode === 'IK'
                                    ? 'bg-green-600 text-white hover:bg-green-700'
                                    : 'bg-gray-800/90 text-gray-300 hover:bg-gray-700'
                                    }`}
                            >
                                <Target className="w-4 h-4" />
                                IK TARGET
                            </button>
                            {/* Fullscreen button */}
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
                                    <LiveRobotScene
                                        joints={joints}
                                        ikMode={jointControlMode === 'IK'}
                                        ikTarget={ikTarget}
                                        onIKTargetDrag={handleIKTargetDrag}
                                        ikReachable={ikReachable}
                                    />
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
                                <div><span className="text-gray-500">X:</span><span className="ml-1 text-white font-mono">{lastCommandedPose?.x?.toFixed(2) || '0.00'}</span></div>
                                <div><span className="text-gray-500">Y:</span><span className="ml-1 text-white font-mono">{lastCommandedPose?.y?.toFixed(2) || '0.00'}</span></div>
                                <div><span className="text-gray-500">Z:</span><span className="ml-1 text-white font-mono">{lastCommandedPose?.z?.toFixed(2) || '0.00'}</span></div>
                            </div>
                        </div>
                        <p className="text-gray-500 text-xs mt-3 text-center">Commanded Target (IK)</p>
                    </div>

                    {/* Sequence Programmer Panel - NEW */}
                    <div className="bg-gray-900 border border-gray-800 rounded-xl p-4 flex flex-col h-[400px]">
                        <h3 className="text-white font-semibold mb-4 flex items-center gap-2">
                            <List className="w-4 h-4 text-cyan-400" />
                            Sequence Programmer
                        </h3>

                        {/* Controls */}
                        <div className="flex gap-2 mb-3">
                            <button
                                onClick={handleTorqueToggle}
                                className={`flex-1 p-2 rounded-lg flex items-center justify-center gap-2 text-xs font-bold transition-colors ${isTorqueEnabled
                                        ? 'bg-green-600/20 text-green-400 border border-green-600/50 hover:bg-green-600/30'
                                        : 'bg-red-600/20 text-red-400 border border-red-600/50 hover:bg-red-600/30'
                                    }`}
                                title={isTorqueEnabled ? "Motors Enabled" : "Motors Released (Free)"}
                            >
                                <Power className="w-3 h-3" />
                                {isTorqueEnabled ? 'ON' : 'OFF'}
                            </button>

                            <button
                                onClick={handleRecordToggle}
                                className={`flex-1 rounded-lg p-2 text-xs font-bold flex items-center justify-center gap-2 transition-colors border ${isRecording
                                        ? 'bg-red-600/20 text-red-500 border-red-500/50 animate-pulse'
                                        : 'bg-gray-800 hover:bg-gray-700 text-white border-gray-700'
                                    }`}
                                title={isRecording ? "Stop Recording" : "Start Motion Recording"}
                            >
                                {isRecording ? <Square className="w-3 h-3 fill-current" /> : <Circle className="w-3 h-3 fill-red-500 text-red-500" />}
                                {isRecording ? 'STOP' : 'REC'}
                            </button>

                            <button
                                onClick={() => handlePlaySequence()}
                                disabled={sequenceSteps.length === 0 || isRecording}
                                className={`flex-1 rounded-lg p-2 text-xs font-bold flex items-center justify-center gap-2 transition-colors border ${isPlaying
                                        ? 'bg-amber-600 border-amber-500 text-white animate-pulse'
                                        : sequenceSteps.length === 0 || isRecording
                                            ? 'bg-gray-800 border-gray-700 text-gray-500 cursor-not-allowed'
                                            : 'bg-blue-600 border-blue-500 hover:bg-blue-500 text-white'
                                    }`}
                            >
                                {isPlaying ? <Pause className="w-3 h-3" /> : <Play className="w-3 h-3" />}
                                {isPlaying ? 'STOP' : 'PLAY'}
                            </button>

                            <button
                                onClick={handleClearSequence}
                                disabled={sequenceSteps.length === 0 || isRecording}
                                className="px-3 bg-gray-800 hover:bg-red-900/30 text-gray-400 hover:text-red-400 rounded-lg border border-gray-700 transition-colors"
                            >
                                <Trash2 className="w-3 h-3" />
                            </button>
                        </div>

                        {/* Step Count Badge */}
                        <div className="text-xs text-gray-500 mb-2 px-1 flex justify-between">
                            <span>{sequenceSteps.length} MOTION SEQUENCES</span>
                            {isRecording && <span className="text-red-400 animate-pulse text-[10px]">● RECORDING...</span>}
                        </div>

                        {/* Step List */}
                        <div className="flex-1 overflow-y-auto min-h-0 bg-black/20 rounded-lg p-2 space-y-1 border border-gray-800 custom-scrollbar">
                            {sequenceSteps.length === 0 ? (
                                <div className="h-full flex flex-col items-center justify-center text-gray-600 text-xs italic opacity-60">
                                    <List className="w-8 h-8 mb-2 opacity-20" />
                                    <span>No sequences recorded</span>
                                </div>
                            ) : (
                                sequenceSteps.map((seq, idx) => (
                                    <div key={seq.id} className="group flex items-center gap-2 bg-gray-800/40 hover:bg-gray-800/60 p-1.5 rounded border border-gray-800/50 hover:border-gray-700 transition-all text-[10px] font-mono">
                                        <div className="bg-gray-700 text-gray-300 w-5 h-5 flex items-center justify-center rounded text-[9px] font-bold">
                                            {idx + 1}
                                        </div>
                                        <div className="flex-1">
                                            <div className="flex justify-between text-gray-400 mb-1">
                                                <span className="text-cyan-400 font-bold">Motion {idx + 1}</span>
                                                <span>{(seq.duration / 1000).toFixed(1)}s / {seq.frameCount}pts</span>
                                            </div>
                                            <div className="grid grid-cols-6 gap-0.5 text-gray-500 text-[9px]">
                                                {/* Show snapshot (first frame) for reference */}
                                                <div title="Base">B:{seq.snapshot.base.toFixed(0)}</div>
                                                <div title="Shoulder">S:{seq.snapshot.shoulder.toFixed(0)}</div>
                                                <div title="Elbow">E:{seq.snapshot.elbow.toFixed(0)}</div>
                                                <div title="WristPitch">P:{seq.snapshot.wristPitch.toFixed(0)}</div>
                                                <div title="WristRoll">R:{seq.snapshot.wristRoll.toFixed(0)}</div>
                                                <div title="Gripper">G:{seq.snapshot.gripper.toFixed(0)}</div>
                                            </div>
                                        </div>
                                        <button
                                            onClick={() => handleDeleteStep(seq.id)}
                                            className="opacity-0 group-hover:opacity-100 p-1 text-gray-500 hover:text-red-400 transition-opacity"
                                        >
                                            <Trash2 className="w-3 h-3" />
                                        </button>
                                    </div>
                                ))
                            )}
                        </div>
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
