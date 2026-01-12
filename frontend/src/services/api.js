// src/services/api.js
import axios from 'axios'
import { rosClient } from './rosClient'

// Varsayılan olarak backend host portu 8082'ye (docker-compose varsayılanı) işaret et
const API_BASE_URL = 'http://localhost:8082/api'

// --- Axios instance ---
export const api = axios.create({
    baseURL: API_BASE_URL,
    headers: { 'Content-Type': 'application/json' },
})

// --- Interceptors ---
api.interceptors.request.use(
    (config) => {
        const token = localStorage.getItem('auth_token')
        if (token) config.headers.Authorization = `Bearer ${token}`
        return config
    },
    (error) => Promise.reject(error)
)

api.interceptors.response.use(
    (response) => response.data,
    (error) => {
        const message = error.response?.data?.message || error.message || 'An error occurred'
        console.error('API Error:', message)
        return Promise.reject(error)
    }
)

// --- Helpers ---
const pickArray = (d, keys = ['data', 'items', 'maps', 'list', 'results']) => {
    if (Array.isArray(d)) return d
    for (const k of keys) {
        if (Array.isArray(d?.[k])) return d[k]
    }
    return []
}

// --- Simulation APIs ---
export const simulationAPI = {
    start: (data) => api.post('/sim/start', data),
    stop: () => api.post('/sim/stop'),
    status: () => api.get('/sim/status'),
}

// --- Examples APIs ---
export const examplesAPI = {
    list: async () => {
        const d = await api.get('/examples')
        return pickArray(d)
    },
    launch: (id) => api.post(`/examples/${id}/launch`),
}

// --- Teleop APIs ---
export const teleopAPI = {
    sendTwist: (data) => api.post('/teleop/twist', data),
}

// --- Navigation APIs ---
export const navigationAPI = {
    sendGoal: (data) => api.post('/nav/goal', data),
    // NOTE: sendWaypoints and getStatus are not implemented in backend
    executeMission: async (waypoints) => {
        console.log('[navigationAPI] executeMission called with waypoints:', waypoints)
        for (let i = 0; i < waypoints.length; i += 1) {
            const wp = waypoints[i]
            console.log(`[navigationAPI] Reaching Waypoint ${i + 1}...`, wp)
            // Her waypoint için 2 saniyelik mock bekleme
            // eslint-disable-next-line no-await-in-loop
            await new Promise((resolve) => setTimeout(resolve, 2000))
        }
        console.log('[navigationAPI] Mission execution completed')
        return { ok: true }
    },
}

// --- Map APIs ---
export const mapAPI = {
    save: (data) => api.post('/map/save', data),

    // Her zaman array döner
    list: async () => {
        const d = await api.get('/map/list')
        return pickArray(d)
    },

    load: (id) => api.post(`/map/load/${id}`),
    delete: (id) => api.delete(`/map/${id}`),
}

// NOTE: configAPI, slamAPI, and sensorAPI have been removed.
// These endpoints do not exist in the backend.
// If needed, implement corresponding backend controllers first.

// --- Visualization APIs ---
export const visualizationAPI = {
    getRvizUrl: () => api.get('/visualization/rviz'),
    getTurtlesimUrl: () => api.get('/visualization/turtlesim'),
}

// --- Turtlesim APIs ---
export const turtlesimAPI = {
    getStatus: () => api.get('/turtlesim/status'),
    sendCmdVel: (data) => api.post('/turtlesim/cmd_vel', data),
}

// --- Health APIs ---
export const healthAPI = {
    getSummary: () => api.get('/health/summary'),
    clearError: () => api.post('/health/clear-error'),
    ping: () => api.get('/health/ping'),
}

// --- ROSBag Recording APIs (mock) ---
export const rosbagAPI = {
    startRecording: () => {
        console.log('[rosbagAPI] startRecording called')
        return new Promise((resolve) => {
            setTimeout(() => {
                console.log('[rosbagAPI] recording started')
                resolve({ ok: true })
            }, 500)
        })
    },
    stopRecording: () => {
        console.log('[rosbagAPI] stopRecording called')
        return new Promise((resolve) => {
            setTimeout(() => {
                const payload = {
                    filename: 'session_2024_01.bag',
                    size: '45MB',
                }
                console.log('[rosbagAPI] recording stopped', payload)
                resolve(payload)
            }, 500)
        })
    },
}

// --- Arm APIs (real ROS publishing) ---
export const armAPI = {
    sendJoints: (joints) => {
        if (!rosClient.isConnected()) {
            return Promise.reject(new Error('ROS Bridge not connected'))
        }

        const payload = {
            mode: 'JOINTS',
            data: joints,
        }

        try {
            rosClient.publishTopic('/so_arm/command', 'std_msgs/String', {
                data: JSON.stringify(payload),
            })
            return Promise.resolve({ ok: true })
        } catch (e) {
            return Promise.reject(e)
        }
    },
    sendPose: (pose) => {
        if (!rosClient.isConnected()) {
            return Promise.reject(new Error('ROS Bridge not connected'))
        }

        const payload = {
            mode: 'IK',
            data: pose,
        }

        try {
            rosClient.publishTopic('/so_arm/command', 'std_msgs/String', {
                data: JSON.stringify(payload),
            })
            return Promise.resolve({ ok: true })
        } catch (e) {
            return Promise.reject(e)
        }
    },
}

export default api
