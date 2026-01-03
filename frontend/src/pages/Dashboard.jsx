import React, { useState } from 'react'
import { motion } from 'framer-motion'
import { useNavigate } from 'react-router-dom'
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query'
import toast from 'react-hot-toast'
import {
    Activity,
    Bot,
    Cpu,
    Map,
    Play,
    Square,
    Zap,
    Clock,
    TrendingUp,
    Battery,
    Wifi,
    HardDrive,
    AlertTriangle,
    CheckCircle,
    XCircle,
    RefreshCw,
    ExternalLink,
    FileText,
    RotateCcw,
    X,
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import { simulationAPI, mapAPI, examplesAPI, healthAPI } from '../services/api'

// Color mapping - Tailwind safe classes
const colorClasses = {
    primary: {
        bg: 'bg-blue-500/10',
        text: 'text-blue-500',
    },
    green: {
        bg: 'bg-green-500/10',
        text: 'text-green-500',
    },
    blue: {
        bg: 'bg-blue-500/10',
        text: 'text-blue-500',
    },
    purple: {
        bg: 'bg-purple-500/10',
        text: 'text-purple-500',
    },
    orange: {
        bg: 'bg-orange-500/10',
        text: 'text-orange-500',
    },
    gray: {
        bg: 'bg-gray-500/10',
        text: 'text-gray-500',
    },
    red: {
        bg: 'bg-red-500/10',
        text: 'text-red-500',
    },
    yellow: {
        bg: 'bg-yellow-500/10',
        text: 'text-yellow-500',
    },
}

const StatCard = ({ icon: Icon, title, value, unit, trend, color = 'primary', onClick }) => {
    const colors = colorClasses[color] || colorClasses.primary

    return (
        <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            whileHover={{ scale: onClick ? 1.02 : 1 }}
            onClick={onClick}
            className={`bg-gray-900 border border-gray-800 rounded-xl p-6 ${onClick ? 'cursor-pointer' : ''}`}
        >
            <div className="flex items-center justify-between mb-4">
                <div className={`p-3 ${colors.bg} rounded-lg`}>
                    <Icon className={`w-6 h-6 ${colors.text}`} />
                </div>
                {trend && (
                    <div className="flex items-center gap-1 text-green-500 text-sm">
                        <TrendingUp className="w-4 h-4" />
                        <span>{trend}</span>
                    </div>
                )}
            </div>
            <div>
                <p className="text-gray-400 text-sm mb-1">{title}</p>
                <p className="text-2xl font-bold text-white">
                    {value}
                    {unit && <span className="text-lg text-gray-400 ml-1">{unit}</span>}
                </p>
            </div>
        </motion.div>
    )
}

const Dashboard = () => {
    const navigate = useNavigate()
    const queryClient = useQueryClient()

    // Single source of truth: health summary
    const { data: health, isLoading: healthLoading } = useQuery({
        queryKey: ['health-summary'],
        queryFn: healthAPI.getSummary,
        refetchInterval: 3000, // Refresh every 3 seconds
    })

    const { data: maps } = useQuery({
        queryKey: ['maps'],
        queryFn: mapAPI.list,
    })

    const { data: examples } = useQuery({
        queryKey: ['examples'],
        queryFn: examplesAPI.list,
    })

    // Smart simulation controls
    const startSimulation = useMutation({
        mutationFn: simulationAPI.start,
        onSuccess: () => {
            toast.success('Simulation started')
            queryClient.invalidateQueries(['health-summary'])
            navigate('/simulator')
        },
        onError: (error) => {
            toast.error('Failed to start simulation')
            console.error(error)
        },
    })

    const stopSimulation = useMutation({
        mutationFn: simulationAPI.stop,
        onSuccess: () => {
            toast.success('Simulation stopped')
            queryClient.invalidateQueries(['health-summary'])
        },
        onError: (error) => {
            toast.error('Failed to stop simulation')
            console.error(error)
        },
    })

    const clearError = useMutation({
        mutationFn: healthAPI.clearError,
        onSuccess: () => {
            toast.success('Error cleared')
            queryClient.invalidateQueries(['health-summary'])
        },
    })

    // Get simulation status from health summary
    const simStatus = health?.data?.activeSimulation?.status || 'STOPPED'
    const overallStatus = health?.data?.overallStatus || 'UNKNOWN'
    const lastError = health?.data?.lastError

    // Status colors
    const getStatusColor = (status) => {
        switch (status) {
            case 'HEALTHY':
            case 'RUNNING':
            case 'CONNECTED':
                return 'green'
            case 'DEGRADED':
            case 'STARTING':
                return 'yellow'
            case 'ERROR':
            case 'STOPPED':
            case 'DISCONNECTED':
                return 'red'
            default:
                return 'gray'
        }
    }

    // Service health indicator
    const ServiceHealthBadge = ({ service, label }) => {
        if (!service) return null

        const status = service.status || 'UNKNOWN'
        const color = getStatusColor(status)
        const colors = colorClasses[color]

        return (
            <div className="flex items-center gap-3">
                <div className={`w-3 h-3 rounded-full ${
                    color === 'green' ? 'bg-green-500' :
                    color === 'yellow' ? 'bg-yellow-500 animate-pulse' :
                    color === 'red' ? 'bg-red-500' :
                    'bg-gray-500'
                }`} />
                <div className="flex-1">
                    <p className="text-white text-sm font-medium">{label}</p>
                    <p className="text-gray-500 text-xs">{service.message || status}</p>
                </div>
                {service.url && (
                    <a
                        href={service.url}
                        target="_blank"
                        rel="noopener noreferrer"
                        className="text-gray-400 hover:text-blue-400 transition-colors"
                        title="Open in new tab"
                    >
                        <ExternalLink className="w-3 h-3" />
                    </a>
                )}
            </div>
        )
    }

    // Simulation control button (state-aware)
    const SimulationControl = () => {
        const isLoading = startSimulation.isPending || stopSimulation.isPending

        if (simStatus === 'RUNNING') {
            return (
                <div className="flex gap-2">
                    <motion.button
                        whileTap={{ scale: 0.95 }}
                        onClick={() => stopSimulation.mutate()}
                        disabled={isLoading}
                        className="flex items-center gap-2 px-4 py-2 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all"
                    >
                        {isLoading ? <RefreshCw className="w-4 h-4 animate-spin" /> : <Square className="w-4 h-4" />}
                        {isLoading ? 'Stopping...' : 'Stop Simulation'}
                    </motion.button>
                    <motion.button
                        whileTap={{ scale: 0.95 }}
                        onClick={() => {
                            stopSimulation.mutate()
                            setTimeout(() => startSimulation.mutate(), 1000)
                        }}
                        disabled={isLoading}
                        className="flex items-center gap-2 px-4 py-2 bg-orange-600 hover:bg-orange-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all"
                    >
                        <RotateCcw className="w-4 h-4" />
                        Restart
                    </motion.button>
                </div>
            )
        }

        if (simStatus === 'ERROR') {
            return (
                <motion.button
                    whileTap={{ scale: 0.95 }}
                    onClick={() => queryClient.invalidateQueries(['health-summary'])}
                    className="flex items-center gap-2 px-4 py-2 bg-yellow-600 hover:bg-yellow-700 text-white rounded-lg transition-all"
                >
                    <RefreshCw className="w-4 h-4" />
                    Reconnect
                </motion.button>
            )
        }

        // STOPPED
        return (
            <motion.button
                whileTap={{ scale: 0.95 }}
                onClick={() => startSimulation.mutate()}
                disabled={isLoading}
                className="flex items-center gap-2 px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:cursor-not-allowed text-white rounded-lg transition-all"
            >
                {isLoading ? <RefreshCw className="w-4 h-4 animate-spin" /> : <Play className="w-4 h-4" />}
                {isLoading ? 'Starting...' : 'Start Simulation'}
            </motion.button>
        )
    }

    const quickActions = [
        {
            title: 'Simulator',
            description: 'Control TurtleBot3 simulation',
            icon: Play,
            color: 'green',
            action: () => navigate('/simulator'),
        },
        {
            title: 'View Examples',
            description: 'Browse pre-configured scenarios',
            icon: Cpu,
            color: 'purple',
            action: () => navigate('/examples'),
        },
        {
            title: 'Manage Maps',
            description: 'Load or create new maps',
            icon: Map,
            color: 'orange',
            action: () => navigate('/maps'),
        },
    ]

    return (
        <PageContainer
            title="Dashboard"
            description="Monitor and control your TurtleBot3 simulation"
            actions={<SimulationControl />}
        >
            <div className="space-y-6">
                {/* System Status Overview */}
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                    <StatCard
                        icon={Activity}
                        title="Overall Status"
                        value={overallStatus}
                        color={getStatusColor(overallStatus)}
                    />
                    <StatCard
                        icon={Zap}
                        title="Simulation"
                        value={simStatus}
                        color={getStatusColor(simStatus)}
                    />
                    <StatCard
                        icon={Wifi}
                        title="ROSBridge"
                        value={health?.data?.rosbridge?.status || 'Unknown'}
                        color={getStatusColor(health?.data?.rosbridge?.status)}
                    />
                    <StatCard
                        icon={Activity}
                        title="WebSocket"
                        value={health?.data?.stomp?.status || 'Unknown'}
                        color={getStatusColor(health?.data?.stomp?.status)}
                    />
                </div>

                {/* Last Error Card (only shown if error exists) */}
                {lastError && (
                    <motion.div
                        initial={{ opacity: 0, y: -20 }}
                        animate={{ opacity: 1, y: 0 }}
                        className="bg-red-900/20 border border-red-800 rounded-xl p-4"
                    >
                        <div className="flex items-start justify-between">
                            <div className="flex items-start gap-3 flex-1">
                                <div className="p-2 bg-red-900/50 rounded-lg mt-0.5">
                                    <AlertTriangle className="w-5 h-5 text-red-400" />
                                </div>
                                <div className="flex-1">
                                    <div className="flex items-center gap-2 mb-1">
                                        <h3 className="text-white font-semibold">Last Error</h3>
                                        <span className="px-2 py-0.5 bg-red-900/50 text-red-300 text-xs rounded">
                                            {lastError.source}
                                        </span>
                                    </div>
                                    <p className="text-red-200 text-sm mb-2">{lastError.message}</p>
                                    {lastError.suggestion && (
                                        <div className="flex items-start gap-2 p-2 bg-red-900/30 rounded text-xs text-red-300">
                                            <FileText className="w-3 h-3 mt-0.5 flex-shrink-0" />
                                            <span>{lastError.suggestion}</span>
                                        </div>
                                    )}
                                    {lastError.timestamp && (
                                        <p className="text-red-400 text-xs mt-2">
                                            {new Date(lastError.timestamp).toLocaleString()}
                                        </p>
                                    )}
                                </div>
                            </div>
                            <button
                                onClick={() => clearError.mutate()}
                                className="text-red-400 hover:text-red-300 transition-colors p-1"
                                title="Clear error"
                            >
                                <X className="w-4 h-4" />
                            </button>
                        </div>
                    </motion.div>
                )}

                {/* Quick Actions */}
                <div>
                    <h2 className="text-lg font-semibold text-white mb-4">Quick Actions</h2>
                    <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                        {quickActions.map((action, index) => {
                            const colors = colorClasses[action.color] || colorClasses.primary

                            return (
                                <motion.button
                                    key={action.title}
                                    initial={{ opacity: 0, y: 20 }}
                                    animate={{ opacity: 1, y: 0 }}
                                    transition={{ delay: index * 0.1 }}
                                    whileHover={{ scale: 1.02, y: -2 }}
                                    whileTap={{ scale: 0.98 }}
                                    onClick={action.action}
                                    className="bg-gray-900 border border-gray-800 rounded-xl p-6 text-left hover:border-gray-700 transition-all"
                                >
                                    <div className={`inline-flex p-3 ${colors.bg} rounded-lg mb-4`}>
                                        <action.icon className={`w-6 h-6 ${colors.text}`} />
                                    </div>
                                    <h3 className="text-white font-semibold mb-2">{action.title}</h3>
                                    <p className="text-gray-400 text-sm">{action.description}</p>
                                </motion.button>
                            )
                        })}
                    </div>
                </div>

                {/* Recent Activity */}
                <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                    {/* Recent Maps */}
                    <motion.div
                        initial={{ opacity: 0, x: -20 }}
                        animate={{ opacity: 1, x: 0 }}
                        className="bg-gray-900 border border-gray-800 rounded-xl p-6"
                    >
                        <div className="flex items-center justify-between mb-4">
                            <h3 className="text-white font-semibold">Recent Maps</h3>
                            <button
                                onClick={() => navigate('/maps')}
                                className="text-blue-400 hover:text-blue-300 text-sm"
                            >
                                View all
                            </button>
                        </div>
                        <div className="space-y-3">
                            {maps?.slice(0, 3).map((map) => (
                                <div
                                    key={map.id}
                                    className="flex items-center justify-between p-3 bg-gray-800/50 rounded-lg"
                                >
                                    <div className="flex items-center gap-3">
                                        <Map className="w-4 h-4 text-orange-400" />
                                        <div>
                                            <p className="text-white text-sm font-medium">{map.name}</p>
                                            <p className="text-gray-500 text-xs">
                                                {map.createdAt ? new Date(map.createdAt).toLocaleDateString() : 'Unknown'}
                                            </p>
                                        </div>
                                    </div>
                                    <button className="text-gray-400 hover:text-white">
                                        <Zap className="w-4 h-4" />
                                    </button>
                                </div>
                            ))}
                            {(!maps || maps.length === 0) && (
                                <p className="text-gray-500 text-sm text-center py-4">No maps available</p>
                            )}
                        </div>
                    </motion.div>

                    {/* Available Examples */}
                    <motion.div
                        initial={{ opacity: 0, x: 20 }}
                        animate={{ opacity: 1, x: 0 }}
                        className="bg-gray-900 border border-gray-800 rounded-xl p-6"
                    >
                        <div className="flex items-center justify-between mb-4">
                            <h3 className="text-white font-semibold">Popular Examples</h3>
                            <button
                                onClick={() => navigate('/examples')}
                                className="text-blue-400 hover:text-blue-300 text-sm"
                            >
                                View all
                            </button>
                        </div>
                        <div className="space-y-3">
                            {examples?.slice(0, 3).map((example) => (
                                <div
                                    key={example.id}
                                    className="flex items-center justify-between p-3 bg-gray-800/50 rounded-lg cursor-pointer hover:bg-gray-800 transition-colors"
                                    onClick={() => navigate('/examples')}
                                >
                                    <div className="flex items-center gap-3">
                                        <Bot className="w-4 h-4 text-purple-400" />
                                        <div>
                                            <p className="text-white text-sm font-medium">{example.title}</p>
                                            <p className="text-gray-500 text-xs">{example.category}</p>
                                        </div>
                                    </div>
                                    <span className={`px-2 py-1 text-xs rounded-full ${
                                        example.difficulty === 'EASY' ? 'bg-green-900/50 text-green-400' :
                                        example.difficulty === 'MEDIUM' ? 'bg-yellow-900/50 text-yellow-400' :
                                        'bg-red-900/50 text-red-400'
                                    }`}>
                                        {example.difficulty}
                                    </span>
                                </div>
                            ))}
                            {(!examples || examples.length === 0) && (
                                <p className="text-gray-500 text-sm text-center py-4">No examples available</p>
                            )}
                        </div>
                    </motion.div>
                </div>

                {/* System Health Details */}
                <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    className="bg-gray-900 border border-gray-800 rounded-xl p-6"
                >
                    <h3 className="text-white font-semibold mb-4">System Health</h3>
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                        <ServiceHealthBadge service={health?.data?.rosbridge} label="ROS Bridge" />
                        <ServiceHealthBadge service={health?.data?.stomp} label="WebSocket (STOMP)" />
                        <ServiceHealthBadge service={health?.data?.database} label="Database" />
                        <ServiceHealthBadge service={health?.data?.rvizNovnc} label="RViz Viewer" />
                        <ServiceHealthBadge service={health?.data?.turtlesimNovnc} label="Turtlesim Viewer" />
                        <div className="flex items-center gap-3">
                            <div className={`w-3 h-3 rounded-full bg-blue-500`} />
                            <div className="flex-1">
                                <p className="text-white text-sm font-medium">Backend API</p>
                                <p className="text-gray-500 text-xs">Operational</p>
                            </div>
                        </div>
                    </div>
                </motion.div>
            </div>
        </PageContainer>
    )
}

export default Dashboard
