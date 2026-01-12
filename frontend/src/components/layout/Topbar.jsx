import React, { useState } from 'react'
import { motion } from 'framer-motion'
import { Bell, Search, User, Power, Info, GitBranch, Disc } from 'lucide-react'
import { useQuery } from '@tanstack/react-query'
import toast from 'react-hot-toast'
import { simulationAPI, rosbagAPI } from '../../services/api'

const Topbar = () => {
    const { data: status } = useQuery({
        queryKey: ['simulation-status'],
        queryFn: simulationAPI.status,
        refetchInterval: 5000,
    })

    const [isRecording, setIsRecording] = useState(false)
    const [isBusy, setIsBusy] = useState(false)

    const handleToggleRecording = async () => {
        if (isBusy) return
        setIsBusy(true)

        try {
            if (!isRecording) {
                await rosbagAPI.startRecording()
                setIsRecording(true)
                toast.success('Recording started')
            } else {
                const result = await rosbagAPI.stopRecording()
                setIsRecording(false)
                toast.success(`Bag saved: ${result.filename}`)
            }
        } catch (e) {
            console.error('[Topbar] Recording toggle failed', e)
            toast.error('Recording action failed')
        } finally {
            setIsBusy(false)
        }
    }

    const currentTime = new Date().toLocaleTimeString('en-US', {
        hour: '2-digit',
        minute: '2-digit',
    })

    return (
        <header className="h-16 bg-gray-900 border-b border-gray-800 px-6 flex items-center justify-between">
            {/* Left Section - Search */}
            <div className="flex items-center gap-4 flex-1">
                <div className="relative max-w-md w-full">
                    <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-500" />
                    <input
                        type="text"
                        placeholder="Search commands, examples, or documentation..."
                        className="w-full pl-10 pr-4 py-2 bg-gray-800 border border-gray-700 rounded-lg text-sm text-gray-300 placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-primary-500 focus:border-transparent"
                    />
                </div>
            </div>

            {/* Center Section - Status */}
            <div className="flex items-center gap-6">
                <motion.div
                    className="flex items-center gap-3 px-4 py-2 bg-gray-800 rounded-lg"
                    initial={{ opacity: 0, y: -10 }}
                    animate={{ opacity: 1, y: 0 }}
                >
                    <div className="flex items-center gap-2">
                        <GitBranch className="w-4 h-4 text-primary-400" />
                        <span className="text-sm font-medium text-gray-300">ROS2 Humble</span>
                    </div>
                    <div className="w-px h-4 bg-gray-700" />
                    <div className="flex items-center gap-2">
                        {status?.status === 'RUNNING' ? (
                            <>
                                <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
                                <span className="text-sm font-medium text-green-400">Simulation Active</span>
                            </>
                        ) : (
                            <>
                                <div className="w-2 h-2 bg-gray-500 rounded-full" />
                                <span className="text-sm font-medium text-gray-400">Simulation Idle</span>
                            </>
                        )}
                    </div>
                </motion.div>
            </div>

            {/* Right Section - Actions */}
            <div className="flex items-center gap-3">
                <span className="text-sm text-gray-400">{currentTime}</span>

                {/* Record Session Button */}
                <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    onClick={handleToggleRecording}
                    disabled={isBusy}
                    className={`flex items-center gap-2 px-3 py-2 rounded-lg border transition-colors text-sm
                        ${isRecording
                            ? 'border-red-500/40 bg-red-500/10 text-red-400'
                            : 'border-gray-700 bg-gray-800 text-gray-300 hover:bg-gray-700'
                        } disabled:opacity-60 disabled:cursor-not-allowed`}
                >
                    <Disc
                        className={`w-4 h-4 ${isRecording ? 'text-red-400 animate-pulse' : 'text-gray-300'}`}
                        fill={isRecording ? 'currentColor' : 'none'}
                    />
                    <span>{isRecording ? 'Recording...' : 'Record'}</span>
                </motion.button>

                <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    className="relative p-2 text-gray-400 hover:text-white hover:bg-gray-800 rounded-lg transition-colors"
                >
                    <Bell className="w-5 h-5" />
                    <span className="absolute top-1 right-1 w-2 h-2 bg-primary-500 rounded-full" />
                </motion.button>

                <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    className="p-2 text-gray-400 hover:text-white hover:bg-gray-800 rounded-lg transition-colors"
                >
                    <Info className="w-5 h-5" />
                </motion.button>

                <div className="w-px h-8 bg-gray-800" />

                <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    className="flex items-center gap-2 px-3 py-2 text-gray-400 hover:text-white hover:bg-gray-800 rounded-lg transition-colors"
                >
                    <div className="w-8 h-8 bg-gradient-to-br from-primary-500 to-primary-600 rounded-full flex items-center justify-center">
                        <User className="w-4 h-4 text-white" />
                    </div>
                    <span className="text-sm font-medium">Admin</span>
                </motion.button>

                <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    className="p-2 text-red-400 hover:text-red-300 hover:bg-red-900/20 rounded-lg transition-colors"
                >
                    <Power className="w-5 h-5" />
                </motion.button>
            </div>
        </header>
    )
}

export default Topbar