import ROSLIB from 'roslib'

const DEFAULT_URL = (import.meta.env && import.meta.env.VITE_ROSBRIDGE_URL) || '/rosbridge'

function toWsUrl(input) {
    const val = (input || DEFAULT_URL).trim()
    if (val.startsWith('/')) {
        const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
        return `${proto}//${window.location.host}${val}`
    }
    if (val.startsWith('http://'))  return 'ws://'  + val.slice(7)
    if (val.startsWith('https://')) return 'wss://' + val.slice(8)
    return val // already ws:// or wss://
}

let ros = null
let connectPromise = null
let state = 'DISCONNECTED' // DISCONNECTED | CONNECTING | CONNECTED
const topics = new Map()
const statusListeners = new Set()
const connectionListeners = new Set() // Sidebar.jsx için
let reconnectAttempts = 0
const maxReconnectAttempts = 5
let reconnectTimer = null
let destroyed = false

function setState(next) {
    if (state === next) return
    const prevState = state
    state = next

    // State change listeners'lara bildir
    for (const cb of statusListeners) {
        try { cb(state) } catch (e) { console.warn('State listener error:', e) }
    }

    // Connection event listeners'lara bildir (Sidebar.jsx için)
    const event = state === 'CONNECTED' ? 'connected' : 'disconnected'
    for (const cb of connectionListeners) {
        try { cb(event) } catch (e) { console.warn('Connection listener error:', e) }
    }
}

function cleanupConnHandlers(target, handlers) {
    if (!target) return
    for (const [evt, fn] of handlers) {
        try {
            const offFn = target.off || target.removeListener
            offFn && offFn.call(target, evt, fn)
        } catch {}
    }
}

function connect(url) {
    if (destroyed) return Promise.reject(new Error('ROS client destroyed'))

    const wsUrl = toWsUrl(url)
    console.log(`[ROS] Attempting to connect to ${wsUrl}`)
    
    if (ros && ros.socket && ros.socket.readyState === 1) {
        console.log('[ROS] Already connected')
        return Promise.resolve(ros)
    }
    
    if (connectPromise) {
        console.log('[ROS] Connection in progress, reusing promise')
        return connectPromise
    }

    setState('CONNECTING')
    ros = new ROSLIB.Ros({ url: wsUrl })

    connectPromise = new Promise((resolve, reject) => {
        const onOpen = () => {
            cleanupConnHandlers(ros, handlers)
            reconnectAttempts = 0
            console.log(`[ROS] CONNECTED to ${wsUrl}`)
            setState('CONNECTED')
            connectPromise = null
            resolve(ros)
        }
        const onError = (e) => {
            cleanupConnHandlers(ros, handlers)
            console.error('[ROS] Connection ERROR:', e)
            setState('DISCONNECTED')
            connectPromise = null
            reject(e)
        }
        const onClose = () => {
            cleanupConnHandlers(ros, handlers)
            console.warn('[ROS] Connection closed')
            setState('DISCONNECTED')
            connectPromise = null
            if (destroyed) {
                console.log('[ROS] Client destroyed, not reconnecting')
                return
            }
            if (reconnectAttempts >= maxReconnectAttempts) {
                console.error('[ROS] Max reconnect attempts reached')
                return
            }
            reconnectAttempts += 1
            const baseDelay = 500 * Math.pow(2, reconnectAttempts - 1)
            const maxDelay = 8000
            const delay = Math.min(baseDelay, maxDelay) + Math.random() * 200
            console.log(`[ROS] RETRY ${reconnectAttempts}/${maxReconnectAttempts} in ${Math.round(delay)}ms`)
            reconnectTimer = setTimeout(() => { 
                if (!destroyed) {
                    connect(wsUrl).catch(() => {}) 
                }
            }, delay)
        }

        const handlers = [
            ['connection', onOpen],
            ['error', onError],
            ['close', onClose],
        ]
        handlers.forEach(([evt, fn]) => ros.on(evt, fn))
    })

    return connectPromise
}

function disconnect() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null }
    topics.forEach((t) => { try { t.unsubscribe?.() } catch {}; try { t.unadvertise?.() } catch {} })
    topics.clear()
    try { ros?.close() } catch {}
    ros = null
    setState('DISCONNECTED')
}

function destroy() {
    destroyed = true
    disconnect()
    statusListeners.clear()
    connectionListeners.clear()
}

function isConnected() {
    return !!(ros && ros.socket && ros.socket.readyState === 1)
}

function getConnectionStatus() {
    return {
        connected: isConnected(),
        reconnectAttempts,
        maxReconnectAttempts,
        state
    }
}

function onStateChange(cb) {
    if (typeof cb === 'function') statusListeners.add(cb)
    return () => statusListeners.delete(cb)
}

// Sidebar.jsx için event listener sistemi
function addListener(cb) {
    if (typeof cb === 'function') {
        connectionListeners.add(cb)
    }
    // Cleanup function döndür
    return () => connectionListeners.delete(cb)
}

function removeListener(cb) {
    connectionListeners.delete(cb)
}

function subscribeTopic(name, messageType, cb, opts = {}) {
    if (!ros) {
        console.error('ROS not connected')
        return null
    }
    const key = `sub:${name}:${messageType}`
    if (topics.has(key)) {
        const t = topics.get(key)
        if (cb) t.subscribe(cb)
        return t
    }
    const t = new ROSLIB.Topic({
        ros,
        name,
        messageType,
        throttle_rate: opts.throttle_rate ?? 100,
    })
    if (cb) t.subscribe(cb)
    topics.set(key, t)
    return t
}

function unsubscribeTopic(name) {
    for (const [key, t] of Array.from(topics.entries())) {
        if (key.startsWith(`sub:${name}:`) || key.startsWith(`pub:${name}:`)) {
            try { t.unsubscribe?.() } catch {}
            try { t.unadvertise?.() } catch {}
            topics.delete(key)
        }
    }
}

function publishTopic(name, messageType, message) {
    if (!ros) {
        console.error('ROS not connected')
        return
    }
    const key = `pub:${name}:${messageType}`
    let t = topics.get(key)
    if (!t) {
        t = new ROSLIB.Topic({ ros, name, messageType })
        try { t.advertise?.() } catch {}
        topics.set(key, t)
    }
    t.publish(new ROSLIB.Message(message))
}

// helpers
function publishCmdVel(linear, angular) {
    publishTopic('/cmd_vel', 'geometry_msgs/Twist', {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
    })
}

function publishGoalPose(x, y, theta) {
    publishTopic('/move_base_simple/goal', 'geometry_msgs/PoseStamped', {
        header: {
            stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 },
            frame_id: 'map',
        },
        pose: {
            position: { x, y, z: 0 },
            orientation: { x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) },
        },
    })
}

function stopRobot() { publishCmdVel(0, 0) }
function moveForward(speed = 0.2)  { publishCmdVel(speed, 0) }
function moveBackward(speed = 0.2) { publishCmdVel(-speed, 0) }
function turnLeft(speed = 0.5)     { publishCmdVel(0, speed) }
function turnRight(speed = 0.5)    { publishCmdVel(0, -speed) }

const api = {
    connect,
    disconnect,
    destroy,
    isConnected,
    getConnectionStatus,
    onStateChange,
    addListener,        // Sidebar.jsx için
    removeListener,     // Sidebar.jsx için
    subscribeTopic,
    unsubscribeTopic,
    publishTopic,
    publishCmdVel,
    publishGoalPose,
    stopRobot,
    moveForward,
    moveBackward,
    turnLeft,
    turnRight,
}

export {
    connect,
    disconnect,
    destroy,
    isConnected,
    getConnectionStatus,
    onStateChange,
    addListener,
    removeListener,
    subscribeTopic,
    unsubscribeTopic,
    publishTopic,
    publishCmdVel,
    publishGoalPose,
    stopRobot,
    moveForward,
    moveBackward,
    turnLeft,
    turnRight,
}
export const rosClient = api
export default api