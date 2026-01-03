import React from 'react'
import { NavLink } from 'react-router-dom'
import { motion } from 'framer-motion'
import {
    LayoutDashboard,
    Cpu,
    Grid3x3,
    Map,
    Settings,
    Wifi,
    WifiOff,
    Bot,
    Zap,
    Activity,
    Turtle,
} from 'lucide-react'
import { rosClient } from '../../services/rosClient'
import { wsService } from '../../services/ws'

// Menu items - sabit renk sınıfları kullanın
const menuItems = [
    {
        path: '/dashboard',
        name: 'Dashboard',
        icon: LayoutDashboard,
        color: 'text-blue-400',
        activeColor: 'text-blue-400',
    },
    {
        path: '/simulator',
        name: 'Simulator',
        icon: Cpu,
        color: 'text-green-400',
        activeColor: 'text-green-400',
    },
    {
        path: '/examples',
        name: 'Examples',
        icon: Grid3x3,
        color: 'text-purple-400',
        activeColor: 'text-purple-400',
    },
    {
        path: '/maps',
        name: 'Maps',
        icon: Map,
        color: 'text-orange-400',
        activeColor: 'text-orange-400',
    },
    {
        path: '/settings',
        name: 'Settings',
        icon: Settings,
        color: 'text-gray-400',
        activeColor: 'text-gray-400',
    },
    {
        path: '/turtlesim',
        name: 'Turtlesim',
        icon: Turtle,
        color: 'text-yellow-400',
        activeColor: 'text-yellow-400',
    },
]

const Sidebar = ({ isCollapsed, setIsCollapsed }) => {
    const [rosConnected, setRosConnected] = React.useState(false)
    const [wsConnected, setWsConnected] = React.useState(false)

    React.useEffect(() => {
        // Bağlantı durumlarını kontrol et
        const checkConnections = () => {
            setRosConnected(rosClient.isConnected())
            setWsConnected(wsService.isConnected())
        }

        // İlk kontrol
        checkConnections()

        // Periyodik kontrol
        const interval = setInterval(checkConnections, 2000)

        // ROS bağlantı durumunu dinle - düzeltilmiş
        const removeRosListener = rosClient.addListener((event) => {
            if (event === 'connected') {
                setRosConnected(true)
            } else if (event === 'disconnected') {
                setRosConnected(false)
            }
        })

        // State değişikliklerini dinle
        const removeStateListener = rosClient.onStateChange((state) => {
            setRosConnected(state === 'CONNECTED')
        })

        // Cleanup
        return () => {
            clearInterval(interval)
            if (removeRosListener) removeRosListener()
            if (removeStateListener) removeStateListener()
        }
    }, [])

    return (
        <motion.aside
            initial={{ x: -300 }}
            animate={{ x: 0, width: isCollapsed ? '80px' : '280px' }}
            transition={{ duration: 0.3, ease: 'easeInOut' }}
            className="h-screen bg-gray-900 border-r border-gray-800 flex flex-col"
        >
            {/* Logo Section */}
            <div className="p-6 border-b border-gray-800">
                <motion.div
                    className="flex items-center gap-3"
                    animate={{ justifyContent: isCollapsed ? 'center' : 'flex-start' }}
                >
                    <div className="relative">
                        <Bot className="w-8 h-8 text-blue-500" />
                        <motion.div
                            className="absolute -top-1 -right-1 w-3 h-3 bg-green-500 rounded-full"
                            animate={{ scale: [1, 1.2, 1] }}
                            transition={{ repeat: Infinity, duration: 2 }}
                        />
                    </div>
                    {!isCollapsed && (
                        <motion.div
                            initial={{ opacity: 0 }}
                            animate={{ opacity: 1 }}
                            transition={{ delay: 0.1 }}
                        >
                            <h1 className="text-xl font-bold text-white">TurtleBot3</h1>
                            <p className="text-xs text-gray-400">Web Simulator</p>
                        </motion.div>
                    )}
                </motion.div>
            </div>

            {/* Connection Status */}
            <div className="px-6 py-4 border-b border-gray-800">
                <div className={`space-y-3 ${isCollapsed ? 'items-center' : ''}`}>
                    {/* ROS Connection */}
                    <motion.div
                        className={`flex items-center gap-2 ${isCollapsed ? 'justify-center' : ''}`}
                        whileHover={{ scale: 1.05 }}
                    >
                        {rosConnected ? (
                            <Wifi className="w-4 h-4 text-green-400" />
                        ) : (
                            <WifiOff className="w-4 h-4 text-red-400" />
                        )}
                        {!isCollapsed && (
                            <span className={`text-xs font-medium ${rosConnected ? 'text-green-400' : 'text-red-400'}`}>
                                ROS {rosConnected ? 'Connected' : 'Disconnected'}
                            </span>
                        )}
                    </motion.div>

                    {/* WebSocket Connection */}
                    <motion.div
                        className={`flex items-center gap-2 ${isCollapsed ? 'justify-center' : ''}`}
                        whileHover={{ scale: 1.05 }}
                    >
                        {wsConnected ? (
                            <Zap className="w-4 h-4 text-green-400" />
                        ) : (
                            <Activity className="w-4 h-4 text-red-400" />
                        )}
                        {!isCollapsed && (
                            <span className={`text-xs font-medium ${wsConnected ? 'text-green-400' : 'text-red-400'}`}>
                                WebSocket {wsConnected ? 'Active' : 'Inactive'}
                            </span>
                        )}
                    </motion.div>
                </div>
            </div>

            {/* Navigation Menu */}
            <nav className="flex-1 p-4 space-y-2">
                {menuItems.map((item) => (
                    <NavLink
                        key={item.path}
                        to={item.path}
                        className={({ isActive }) =>
                            `flex items-center gap-3 px-3 py-3 rounded-lg transition-all duration-200 group
                            ${isActive
                                ? 'bg-blue-600/20 text-blue-400 shadow-lg shadow-blue-500/20'
                                : 'text-gray-400 hover:bg-gray-800 hover:text-white'
                            } ${isCollapsed ? 'justify-center' : ''}`
                        }
                    >
                        {({ isActive }) => (
                            <>
                                <item.icon
                                    className={`w-5 h-5 ${
                                        isActive
                                            ? item.activeColor
                                            : 'group-hover:text-white'
                                    }`}
                                />
                                {!isCollapsed && (
                                    <motion.span
                                        initial={{ opacity: 0 }}
                                        animate={{ opacity: 1 }}
                                        transition={{ delay: 0.1 }}
                                        className="font-medium"
                                    >
                                        {item.name}
                                    </motion.span>
                                )}
                                {!isCollapsed && isActive && (
                                    <motion.div
                                        className="ml-auto w-2 h-2 bg-blue-400 rounded-full"
                                        initial={{ scale: 0 }}
                                        animate={{ scale: 1 }}
                                        transition={{ type: 'spring' }}
                                    />
                                )}
                            </>
                        )}
                    </NavLink>
                ))}
            </nav>

            {/* Collapse Button */}
            <div className="p-4 border-t border-gray-800">
                <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    onClick={() => setIsCollapsed(!isCollapsed)}
                    className={`w-full flex items-center justify-center py-2 px-3 rounded-lg bg-gray-800 hover:bg-gray-700 transition-colors ${
                        isCollapsed ? '' : 'gap-3'
                    }`}
                >
                    <motion.svg
                        animate={{ rotate: isCollapsed ? 180 : 0 }}
                        transition={{ duration: 0.3 }}
                        className="w-5 h-5 text-gray-400"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                    >
                        <path
                            strokeLinecap="round"
                            strokeLinejoin="round"
                            strokeWidth={2}
                            d="M11 19l-7-7 7-7m8 14l-7-7 7-7"
                        />
                    </motion.svg>
                    {!isCollapsed && (
                        <span className="text-sm text-gray-400">Collapse</span>
                    )}
                </motion.button>
            </div>
        </motion.aside>
    )
}

export default Sidebar