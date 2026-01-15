/**
 * SO-ARM101 Isolated ROS Client
 * 
 * This is a DEDICATED client for the SO-ARM101 robot arm.
 * It does NOT share state with the Turtlebot rosClient.js singleton.
 * 
 * Connects exclusively to /rosbridge-soarm (Vite proxy to port 9092)
 */
import ROSLIB from 'roslib'

// Build WebSocket URL from current host using the dedicated proxy route
function buildWsUrl() {
    // RUTHLESS FIX: connect directly to docker port 9090, bypassing vite proxy
    return 'ws://localhost:9090'
}

/**
 * Factory function to create an isolated SO-ARM101 ROS client instance.
 * Each call returns a NEW independent client.
 */
export function createSoarmRosClient() {
    let ros = null
    let state = 'DISCONNECTED' // DISCONNECTED | CONNECTING | CONNECTED
    let reconnectTimer = null
    let reconnectAttempts = 0
    const maxReconnectAttempts = 10
    const topics = new Map()
    const stateListeners = new Set()
    let destroyed = false

    function setState(next) {
        if (state === next) return
        state = next
        for (const cb of stateListeners) {
            try { cb(state) } catch (e) { console.warn('[SoARM-ROS] State listener error:', e) }
        }
    }

    function connect() {
        if (destroyed) return Promise.reject(new Error('Client destroyed'))
        if (ros && ros.socket && ros.socket.readyState === 1) {
            return Promise.resolve(ros)
        }

        const wsUrl = buildWsUrl()
        console.log(`[SoARM-ROS] Connecting to ${wsUrl}`)

        setState('CONNECTING')
        ros = new ROSLIB.Ros({ url: wsUrl })

        return new Promise((resolve, reject) => {
            const onOpen = () => {
                cleanup()
                reconnectAttempts = 0
                console.log(`[SoARM-ROS] CONNECTED to ${wsUrl}`)
                setState('CONNECTED')
                resolve(ros)
            }

            const onError = (e) => {
                cleanup()
                console.error('[SoARM-ROS] Connection ERROR:', e)
                setState('DISCONNECTED')
                scheduleReconnect()
                reject(e)
            }

            const onClose = () => {
                cleanup()
                console.warn('[SoARM-ROS] Connection closed')
                setState('DISCONNECTED')
                if (!destroyed) {
                    scheduleReconnect()
                }
            }

            function cleanup() {
                try { ros?.off('connection', onOpen) } catch { /* ignore */ }
                try { ros?.off('error', onError) } catch { /* ignore */ }
                try { ros?.off('close', onClose) } catch { /* ignore */ }
            }

            ros.on('connection', onOpen)
            ros.on('error', onError)
            ros.on('close', onClose)
        })
    }

    function scheduleReconnect() {
        if (destroyed || reconnectTimer) return
        if (reconnectAttempts >= maxReconnectAttempts) {
            console.error('[SoARM-ROS] Max reconnect attempts reached')
            return
        }

        reconnectAttempts++
        const delay = Math.min(2000 * reconnectAttempts, 10000) + Math.random() * 500
        console.log(`[SoARM-ROS] Reconnect attempt ${reconnectAttempts}/${maxReconnectAttempts} in ${Math.round(delay)}ms`)

        reconnectTimer = setTimeout(() => {
            reconnectTimer = null
            if (!destroyed) {
                connect().catch(() => { /* ignore reconnect errors */ })
            }
        }, delay)
    }

    function disconnect() {
        if (reconnectTimer) {
            clearTimeout(reconnectTimer)
            reconnectTimer = null
        }
        topics.forEach((t) => {
            try { t.unsubscribe?.() } catch { /* ignore */ }
            try { t.unadvertise?.() } catch { /* ignore */ }
        })
        topics.clear()
        try { ros?.close() } catch { /* ignore */ }
        ros = null
        setState('DISCONNECTED')
    }

    function destroy() {
        destroyed = true
        disconnect()
        stateListeners.clear()
    }

    function isConnected() {
        return !!(ros && ros.socket && ros.socket.readyState === 1)
    }

    function getState() {
        return state
    }

    function onStateChange(cb) {
        if (typeof cb === 'function') stateListeners.add(cb)
        return () => stateListeners.delete(cb)
    }

    function getTopic(name, messageType, isPublisher = false) {
        const key = `${isPublisher ? 'pub' : 'sub'}:${name}:${messageType}`
        if (topics.has(key)) return topics.get(key)

        if (!ros) {
            console.error('[SoARM-ROS] Not connected')
            return null
        }

        const topic = new ROSLIB.Topic({
            ros,
            name,
            messageType,
            throttle_rate: 100,
        })

        if (isPublisher) {
            try { topic.advertise?.() } catch { /* ignore */ }
        }

        topics.set(key, topic)
        return topic
    }

    function publishTopic(name, messageType, message) {
        const topic = getTopic(name, messageType, true)
        if (!topic) return false
        topic.publish(new ROSLIB.Message(message))
        return true
    }

    function subscribeTopic(name, messageType, callback) {
        const topic = getTopic(name, messageType, false)
        if (!topic) return null
        if (callback) topic.subscribe(callback)
        return topic
    }

    // ========== SO-ARM101 Specific Commands ==========

    /**
     * Send a target pose to the motion planner
     */
    function sendTargetPose(pose) {
        console.log('[SoARM-ROS] Sending target pose:', pose)
        return publishTopic('/so_arm101/cmd_pose', 'geometry_msgs/msg/Pose', {
            position: {
                x: pose?.x ?? 0.3,
                y: pose?.y ?? 0.0,
                z: pose?.z ?? 0.3,
            },
            orientation: {
                x: pose?.qx ?? 0.0,
                y: pose?.qy ?? 0.707,
                z: pose?.qz ?? 0.0,
                w: pose?.qw ?? 0.707,
            }
        })
    }

    /**
     * Send generic MoveIt command string to wrapper
     * Matches moveit_wrapper.py topic: /so_arm/moveit_cmd
     */
    function sendMoveItString(cmd) {
        console.log(`[SoARM-ROS] Sending MoveIt command: ${cmd}`)
        return publishTopic('/so_arm/moveit_cmd', 'std_msgs/msg/String', { data: cmd })
    }

    function sendPlanCmd() { return sendMoveItString('plan') }
    function sendExecuteCmd() { return sendMoveItString('execute') }
    function sendStopCmd() { return sendMoveItString('stop') }
    function sendHomeCmd() { return sendMoveItString('home') }

    function sendGripperCmd(open) {
        console.log(`[SoARM-ROS] Gripper ${open ? 'OPEN' : 'CLOSE'}`)
        const cmd = open ? 'gripper_open' : 'gripper_close'
        return sendMoveItString(cmd)
    }

    function sendEstop() {
        console.log('[SoARM-ROS] EMERGENCY STOP!')
        // Send both legacy E-Stop and moveit stop
        publishTopic('/so_arm101/estop', 'std_msgs/msg/Bool', { data: true })
        return sendMoveItString('stop')
    }

    return {
        connect,
        disconnect,
        destroy,
        isConnected,
        getState,
        onStateChange,
        publishTopic,
        subscribeTopic,
        // SO-ARM101 specific
        sendTargetPose,
        sendPlanCmd,
        sendExecuteCmd,
        sendStopCmd,
        sendHomeCmd,
        sendGripperCmd,
        sendEstop,
    }
}

export default createSoarmRosClient
