import React, { useState, useEffect, useRef, useCallback } from 'react'
import { useQuery } from '@tanstack/react-query'
import { 
    Keyboard, 
    Wifi, 
    WifiOff, 
    Maximize2, 
    Minimize2,
    ZoomIn,
    ZoomOut,
    Focus,
    Circle,
    Clock
} from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import { turtlesimAPI, visualizationAPI } from '../services/api'

const Turtlesim = () => {
    // State management
    const [velocity, setVelocity] = useState({ linear: 0, angular: 0 })
    const [isActive, setIsActive] = useState(false)
    const [pressedKeys, setPressedKeys] = useState(new Set())
    const [viewMode, setViewMode] = useState(() => localStorage.getItem('turtlesim_view_mode') || 'fit')
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [hasFocus, setHasFocus] = useState(false)
    const [lastCommandTime, setLastCommandTime] = useState(null)
    const [publishState, setPublishState] = useState('stopped') // stopped | active
    
    // Refs
    const intervalRef = useRef(null)
    const lastSentRef = useRef({ linear: 0, angular: 0 })
    const currentCmdRef = useRef({ linear: 0, angular: 0 })
    const containerRef = useRef(null)
    const iframeRef = useRef(null)

    const LINEAR_VEL = 2.0
    const ANGULAR_VEL = 2.0

    // Persist view mode
    useEffect(() => {
        localStorage.setItem('turtlesim_view_mode', viewMode)
    }, [viewMode])

    // Queries
    const { data: status } = useQuery({
        queryKey: ['turtlesim-status'],
        queryFn: turtlesimAPI.getStatus,
        refetchInterval: 2000,
    })

    const { data: visData, isLoading: isVisLoading, error: visError } = useQuery({
        queryKey: ['turtlesim-visualization'],
        queryFn: visualizationAPI.getTurtlesimUrl,
        retry: 3,
        retryDelay: 1000,
    })

    const sendCmdVel = useCallback((linear, angular) => {
        // Only send if values changed
        if (lastSentRef.current.linear === linear && lastSentRef.current.angular === angular) {
            return
        }

        lastSentRef.current = { linear, angular }
        setVelocity({ linear, angular })
        currentCmdRef.current = { linear, angular }
        setLastCommandTime(new Date())
        setPublishState(linear !== 0 || angular !== 0 ? 'active' : 'stopped')

        turtlesimAPI.sendCmdVel({ linear, angular }).catch((error) => {
            console.error('[Turtlesim] Failed to send cmd_vel:', error)
        })
    }, [])

    const stop = useCallback(() => {
        lastSentRef.current = { linear: 0, angular: 0 }
        currentCmdRef.current = { linear: 0, angular: 0 }
        setVelocity({ linear: 0, angular: 0 })
        setPressedKeys(new Set())
        setIsActive(false)
        setPublishState('stopped')
        turtlesimAPI.sendCmdVel({ linear: 0, angular: 0 }).catch(() => {})
    }, [])

    // Continuous publishing loop (10 Hz)
    useEffect(() => {
        if (intervalRef.current) return
        intervalRef.current = setInterval(() => {
            const { linear, angular } = currentCmdRef.current
            if (linear !== 0 || angular !== 0) {
                turtlesimAPI.sendCmdVel({ linear, angular }).catch(() => {})
            }
        }, 100)
        return () => {
            clearInterval(intervalRef.current)
            intervalRef.current = null
        }
    }, [])

    // Keyboard handlers
    const handleKeyDown = useCallback((event) => {
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(event.key)) {
            event.preventDefault()
        }

        const tag = (event.target && event.target.tagName) || ''
        if (['INPUT', 'TEXTAREA', 'SELECT'].includes(tag)) return

        setPressedKeys((prevKeys) => {
            if (prevKeys.has(event.key)) return prevKeys

            const newPressedKeys = new Set(prevKeys)
            newPressedKeys.add(event.key)

            let linear = 0
            let angular = 0

            if (newPressedKeys.has('w') || newPressedKeys.has('W') || newPressedKeys.has('ArrowUp')) {
                linear += LINEAR_VEL
            }
            if (newPressedKeys.has('s') || newPressedKeys.has('S') || newPressedKeys.has('ArrowDown')) {
                linear -= LINEAR_VEL
            }
            if (newPressedKeys.has('a') || newPressedKeys.has('A') || newPressedKeys.has('ArrowLeft')) {
                angular += ANGULAR_VEL
            }
            if (newPressedKeys.has('d') || newPressedKeys.has('D') || newPressedKeys.has('ArrowRight')) {
                angular -= ANGULAR_VEL
            }

            if (linear !== 0 || angular !== 0) {
                setIsActive(true)
                sendCmdVel(linear, angular)
            } else if (event.key === ' ' || event.key === 'Space') {
                stop()
            }

            return newPressedKeys
        })
    }, [sendCmdVel, stop])

    const handleKeyUp = useCallback((event) => {
        if (event.key === ' ' || event.key === 'Space') return

        setPressedKeys((prevKeys) => {
            const newPressedKeys = new Set(prevKeys)
            newPressedKeys.delete(event.key)

            let linear = 0
            let angular = 0

            if (newPressedKeys.has('w') || newPressedKeys.has('W') || newPressedKeys.has('ArrowUp')) {
                linear += LINEAR_VEL
            }
            if (newPressedKeys.has('s') || newPressedKeys.has('S') || newPressedKeys.has('ArrowDown')) {
                linear -= LINEAR_VEL
            }
            if (newPressedKeys.has('a') || newPressedKeys.has('A') || newPressedKeys.has('ArrowLeft')) {
                angular += ANGULAR_VEL
            }
            if (newPressedKeys.has('d') || newPressedKeys.has('D') || newPressedKeys.has('ArrowRight')) {
                angular -= ANGULAR_VEL
            }

            if (linear === 0 && angular === 0) {
                setIsActive(false)
                stop()
            } else {
                sendCmdVel(linear, angular)
            }

            return newPressedKeys
        })
    }, [sendCmdVel, stop])

    // Keyboard focus management
    useEffect(() => {
        const handleFocus = () => setHasFocus(true)
        const handleBlur = () => {
            setHasFocus(false)
            stop()
        }

        window.addEventListener('focus', handleFocus)
        window.addEventListener('blur', handleBlur)
        window.addEventListener('keydown', handleKeyDown)
        window.addEventListener('keyup', handleKeyUp)

        return () => {
            window.removeEventListener('focus', handleFocus)
            window.removeEventListener('blur', handleBlur)
            window.removeEventListener('keydown', handleKeyDown)
            window.removeEventListener('keyup', handleKeyUp)
            sendCmdVel(0, 0)
        }
    }, [handleKeyDown, handleKeyUp, sendCmdVel, stop])

    // Button handlers (mouse/touch)
    const applyCommand = (linear, angular) => {
        currentCmdRef.current = { linear, angular }
        sendCmdVel(linear, angular)
        setIsActive(linear !== 0 || angular !== 0)
    }

    const handlePressStart = (cmd) => () => applyCommand(cmd.linear, cmd.angular)
    const handlePressEnd = () => applyCommand(0, 0)

    // Fullscreen toggle
    const toggleFullscreen = () => {
        if (!document.fullscreenElement) {
            containerRef.current?.requestFullscreen()
            setIsFullscreen(true)
        } else {
            document.exitFullscreen()
            setIsFullscreen(false)
        }
    }

    // Focus view
    const focusView = () => {
        iframeRef.current?.contentWindow?.focus()
        setHasFocus(true)
    }

    const isConnected = status?.connected ?? true
    const isRunning = status?.running ?? true

    // View mode CSS classes
    const getViewModeClass = () => {
        switch (viewMode) {
            case 'fill':
                return 'w-full h-full object-cover'
            case '1:1':
                return 'w-auto h-auto max-w-full max-h-full'
            case 'fit':
            default:
                return 'w-full h-full object-contain'
        }
    }

    return (
        <PageContainer
            title="Turtlesim Demo"
            description="Interactive ROS 2 turtlesim with keyboard and button control"
            actions={
                <div className="flex items-center gap-2">
                    {/* View Mode Selector */}
                    <select
                        value={viewMode}
                        onChange={(e) => setViewMode(e.target.value)}
                        className="px-3 py-1.5 bg-gray-800 border border-gray-700 rounded-lg text-sm text-gray-300 focus:outline-none focus:ring-2 focus:ring-primary-500"
                    >
                        <option value="fit">Fit</option>
                        <option value="fill">Fill</option>
                        <option value="1:1">1:1</option>
                    </select>

                    {/* Fullscreen Toggle */}
                    <button
                        onClick={toggleFullscreen}
                        className="p-2 bg-gray-800 hover:bg-gray-700 text-gray-300 rounded-lg transition-colors"
                        title={isFullscreen ? 'Exit Fullscreen' : 'Enter Fullscreen'}
                    >
                        {isFullscreen ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
                    </button>

                    {/* Focus View Button */}
                    <button
                        onClick={focusView}
                        className="flex items-center gap-2 px-3 py-1.5 bg-blue-600 hover:bg-blue-700 text-white text-sm rounded-lg transition-colors"
                    >
                        <Focus className="w-4 h-4" />
                        Focus View
                    </button>
                </div>
            }
        >
            <div className="space-y-6" ref={containerRef}>
                {/* Live View */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-4 md:p-6 relative overflow-hidden">
                    <div className="flex items-center justify-between mb-4">
                        <h2 className="text-lg font-semibold text-white">Live View</h2>
                        <div className="flex items-center gap-3">
                            {hasFocus && (
                                <div className="flex items-center gap-2 text-green-400 text-xs">
                                    <Circle className="w-2 h-2 fill-current" />
                                    <span>Keyboard Active</span>
                                </div>
                            )}
                            {isVisLoading && (
                                <div className="text-xs text-gray-400">Loading visualization...</div>
                            )}
                            {visError && (
                                <div className="text-xs text-red-400">Failed to load view</div>
                            )}
                        </div>
                    </div>

                    <div
                        className={`relative rounded-lg border-2 overflow-hidden bg-gray-950 ${
                            hasFocus ? 'border-blue-500' : 'border-gray-800'
                        } transition-colors`}
                        style={{ 
                            width: '100%', 
                            aspectRatio: '1 / 1', 
                            maxHeight: viewMode === '1:1' ? 'none' : '70vh' 
                        }}
                    >
                        {(!visData?.url || visError) && (
                            <div className="flex flex-col items-center justify-center h-96 text-gray-400 text-sm gap-3">
                                <WifiOff className="w-12 h-12" />
                                <p>Visualization unavailable</p>
                                {visError && (
                                    <p className="text-xs text-red-400">Error: {visError.message}</p>
                                )}
                            </div>
                        )}
                        {visData?.url && (
                            <>
                                <iframe
                                    ref={iframeRef}
                                    title="Turtlesim Live View"
                                    src={visData.url}
                                    className={`absolute inset-0 ${getViewModeClass()} border-0`}
                                    style={{ background: '#0f172a' }}
                                    allow="clipboard-read; clipboard-write"
                                    onFocus={() => setHasFocus(true)}
                                    onBlur={() => setHasFocus(false)}
                                />
                                
                                {/* Keyboard Focus Overlay (only when NOT focused) */}
                                {!hasFocus && (
                                    <div className="absolute inset-0 bg-black/40 backdrop-blur-[2px] flex items-center justify-center pointer-events-none">
                                        <div className="bg-gray-900/90 border border-gray-700 rounded-xl p-6 max-w-md text-center pointer-events-auto">
                                            <Keyboard className="w-12 h-12 text-blue-400 mx-auto mb-3" />
                                            <h3 className="text-white font-semibold mb-2">Keyboard Control Inactive</h3>
                                            <p className="text-gray-400 text-sm mb-4">
                                                Click inside the simulation view to enable keyboard control
                                            </p>
                                            <button
                                                onClick={focusView}
                                                className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded-lg transition-colors flex items-center gap-2 mx-auto"
                                            >
                                                <Focus className="w-4 h-4" />
                                                Focus View
                                            </button>
                                        </div>
                                    </div>
                                )}
                            </>
                        )}
                        {isVisLoading && (
                            <div className="absolute inset-0 bg-gray-900/70 flex items-center justify-center">
                                <div className="text-center">
                                    <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-500 mx-auto mb-3"></div>
                                    <p className="text-gray-300 text-sm">Connecting to turtlesim...</p>
                                </div>
                            </div>
                        )}
                    </div>
                </div>

                {/* Command Feedback Panel */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-6">
                    <div className="flex items-center justify-between mb-4">
                        <h2 className="text-lg font-semibold text-white">Command Feedback</h2>
                        <div className="flex items-center gap-2">
                            <div className={`w-2 h-2 rounded-full ${publishState === 'active' ? 'bg-green-500 animate-pulse' : 'bg-gray-600'}`} />
                            <span className="text-sm text-gray-400">
                                {publishState === 'active' ? 'Publishing' : 'Stopped'}
                            </span>
                        </div>
                    </div>

                    <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
                        {/* Linear Velocity */}
                        <div className="bg-gray-800/50 rounded-lg p-4">
                            <div className="text-gray-400 text-xs mb-1">Linear Velocity</div>
                            <div className="text-xl font-bold text-white">
                                {velocity.linear.toFixed(2)} <span className="text-xs text-gray-400">m/s</span>
                            </div>
                        </div>

                        {/* Angular Velocity */}
                        <div className="bg-gray-800/50 rounded-lg p-4">
                            <div className="text-gray-400 text-xs mb-1">Angular Velocity</div>
                            <div className="text-xl font-bold text-white">
                                {velocity.angular.toFixed(2)} <span className="text-xs text-gray-400">rad/s</span>
                            </div>
                        </div>

                        {/* Last Command Time */}
                        <div className="bg-gray-800/50 rounded-lg p-4 col-span-2 md:col-span-1">
                            <div className="text-gray-400 text-xs mb-1 flex items-center gap-1">
                                <Clock className="w-3 h-3" />
                                Last Command
                            </div>
                            <div className="text-sm font-mono text-white">
                                {lastCommandTime ? lastCommandTime.toLocaleTimeString() : '--:--:--'}
                            </div>
                        </div>
                    </div>

                    {/* Publishing Status */}
                    {isActive && (
                        <div className="mt-4 flex items-center gap-2 text-green-400">
                            <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse" />
                            <span className="text-sm font-medium">Active keyboard control</span>
                        </div>
                    )}
                </div>

                {/* Status & Connection Panel */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-6">
                    <h2 className="text-lg font-semibold text-white mb-4">Status</h2>
                    <div className="flex items-center gap-6">
                        <div className="flex items-center gap-2">
                            {isConnected ? (
                                <Wifi className="w-5 h-5 text-green-400" />
                            ) : (
                                <WifiOff className="w-5 h-5 text-red-400" />
                            )}
                            <span className={`text-sm ${isConnected ? 'text-green-400' : 'text-red-400'}`}>
                                {isConnected ? 'Connected' : 'Disconnected'}
                            </span>
                        </div>
                        <div className="flex items-center gap-2">
                            <div className={`w-3 h-3 rounded-full ${isRunning ? 'bg-green-500' : 'bg-gray-500'} animate-pulse`} />
                            <span className="text-sm text-gray-400">
                                Turtlesim {isRunning ? 'Running' : 'Stopped'}
                            </span>
                        </div>
                    </div>
                </div>

                {/* Button-Based Control Panel (Reliable Fallback) */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-6">
                    <h2 className="text-lg font-semibold text-white mb-4 flex items-center gap-2">
                        <Keyboard className="w-5 h-5 text-blue-400" />
                        Button Controls (Reliable Fallback)
                    </h2>
                    
                    <p className="text-gray-400 text-sm mb-4">
                        Use these buttons if keyboard control is not working
                    </p>

                    <div className="grid grid-cols-3 gap-3 max-w-md mx-auto">
                        {/* Forward */}
                        <button
                            onMouseDown={handlePressStart({ linear: LINEAR_VEL, angular: 0 })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: LINEAR_VEL, angular: 0 })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 active:bg-blue-600 text-white py-3 px-3 rounded-lg transition-colors col-start-2 font-medium"
                        >
                            ↑ Forward
                        </button>

                        {/* Rotate Left */}
                        <button
                            onMouseDown={handlePressStart({ linear: 0, angular: ANGULAR_VEL })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: 0, angular: ANGULAR_VEL })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 active:bg-blue-600 text-white py-3 px-3 rounded-lg transition-colors font-medium"
                        >
                            ← Left
                        </button>

                        {/* Stop (center) */}
                        <button
                            onClick={handlePressEnd}
                            className="bg-red-700 hover:bg-red-600 active:bg-red-800 text-white py-3 px-3 rounded-lg transition-colors font-bold"
                        >
                            STOP
                        </button>

                        {/* Rotate Right */}
                        <button
                            onMouseDown={handlePressStart({ linear: 0, angular: -ANGULAR_VEL })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: 0, angular: -ANGULAR_VEL })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 active:bg-blue-600 text-white py-3 px-3 rounded-lg transition-colors font-medium"
                        >
                            → Right
                        </button>

                        {/* Backward */}
                        <button
                            onMouseDown={handlePressStart({ linear: -LINEAR_VEL, angular: 0 })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: -LINEAR_VEL, angular: 0 })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 active:bg-blue-600 text-white py-3 px-3 rounded-lg transition-colors col-start-2 font-medium"
                        >
                            ↓ Backward
                        </button>
                    </div>

                    <div className="mt-4 pt-4 border-t border-gray-800 text-center">
                        <p className="text-xs text-gray-500">
                            Press and hold buttons to move. Release to stop.
                        </p>
                    </div>
                </div>

                {/* Keyboard Controls Reference */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-6">
                    <h2 className="text-lg font-semibold text-white mb-4">Keyboard Shortcuts</h2>
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                        <div className="space-y-2">
                            <div className="flex items-center gap-3 text-gray-300">
                                <div className="flex gap-1">
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">W</kbd>
                                    <span className="mx-1">or</span>
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">↑</kbd>
                                </div>
                                <span>Move Forward</span>
                            </div>
                            <div className="flex items-center gap-3 text-gray-300">
                                <div className="flex gap-1">
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">S</kbd>
                                    <span className="mx-1">or</span>
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">↓</kbd>
                                </div>
                                <span>Move Backward</span>
                            </div>
                        </div>
                        <div className="space-y-2">
                            <div className="flex items-center gap-3 text-gray-300">
                                <div className="flex gap-1">
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">A</kbd>
                                    <span className="mx-1">or</span>
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">←</kbd>
                                </div>
                                <span>Rotate Left</span>
                            </div>
                            <div className="flex items-center gap-3 text-gray-300">
                                <div className="flex gap-1">
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">D</kbd>
                                    <span className="mx-1">or</span>
                                    <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">→</kbd>
                                </div>
                                <span>Rotate Right</span>
                            </div>
                        </div>
                        <div className="col-span-full">
                            <div className="flex items-center gap-3 text-gray-300">
                                <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">Space</kbd>
                                <span>Emergency Stop</span>
                            </div>
                        </div>
                    </div>
                </div>

                {/* Info Banner */}
                <div className="bg-blue-900/20 border border-blue-800/50 rounded-xl p-4">
                    <p className="text-sm text-blue-300">
                        <strong>Demo Page:</strong> This is a demonstration of ROS 2 turtlesim. 
                        Commands are published via ROSBridge WebSocket to <code className="px-1 py-0.5 bg-blue-900/50 rounded text-xs">/turtle1/cmd_vel</code>.
                        {!hasFocus && (
                            <span className="block mt-2 text-yellow-300">
                                ⚠️ Click "Focus View" or click inside the simulation to enable keyboard control.
                            </span>
                        )}
                    </p>
                </div>
            </div>
        </PageContainer>
    )
}

export default Turtlesim
