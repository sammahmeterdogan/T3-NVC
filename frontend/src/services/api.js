// src/services/api.js
import axios from 'axios'

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
    sendWaypoints: (data) => api.post('/nav/waypoints', data),
    getStatus: () => api.get('/nav/status'),
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

// --- Config APIs ---
export const configAPI = {
    getModels: () => api.get('/config/models'),
    getRobotConfigs: () => api.get('/config'),
    createConfig: (data) => api.post('/config', data),
    updateConfig: (id, data) => api.put(`/config/${id}`, data),
}

// --- SLAM APIs ---
export const slamAPI = {
    start: () => api.post('/slam/start'),
    stop: () => api.post('/slam/stop'),
    getStatus: () => api.get('/slam/status'),
}

// --- Sensor APIs ---
export const sensorAPI = {
    getScan: () => api.get('/sensors/scan'),
    getOdom: () => api.get('/sensors/odom'),
    getImu: () => api.get('/sensors/imu'),
    getBattery: () => api.get('/sensors/battery'),
}

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

export default api
