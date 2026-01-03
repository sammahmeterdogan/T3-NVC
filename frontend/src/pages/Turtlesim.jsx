import React, { useState, useEffect, useRef, useCallback } from 'react'
import { useQuery } from '@tanstack/react-query'
import { Keyboard, Wifi, WifiOff } from 'lucide-react'
import PageContainer from '../components/layout/PageContainer'
import { turtlesimAPI, visualizationAPI } from '../services/api'

const Turtlesim = () => {
    const [velocity, setVelocity] = useState({ linear: 0, angular: 0 })
    const [isActive, setIsActive] = useState(false)
    const [pressedKeys, setPressedKeys] = useState(new Set())
    const intervalRef = useRef(null)
    const lastSentRef = useRef({ linear: 0, angular: 0 })
    const currentCmdRef = useRef({ linear: 0, angular: 0 })

    const LINEAR_VEL = 2.0
    const ANGULAR_VEL = 2.0

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

        turtlesimAPI.sendCmdVel({ linear, angular }).catch((error) => {
            console.error('Failed to send cmd_vel:', error)
        })
    }, [])

    const stop = useCallback(() => {
        lastSentRef.current = { linear: 0, angular: 0 }
        currentCmdRef.current = { linear: 0, angular: 0 }
        setVelocity({ linear: 0, angular: 0 })
        setPressedKeys(new Set())
        setIsActive(false)
        turtlesimAPI.sendCmdVel({ linear: 0, angular: 0 }).catch(() => {})
    }, [])

    // Continuous publishing loop (10 Hz) for held keys or buttons
    useEffect(() => {
        if (intervalRef.current) return
        intervalRef.current = setInterval(() => {
            const { linear, angular } = currentCmdRef.current
            turtlesimAPI.sendCmdVel({ linear, angular }).catch(() => {})
        }, 100)
        return () => {
            clearInterval(intervalRef.current)
            intervalRef.current = null
        }
    }, [])

    const handleKeyDown = useCallback((event) => {
        // Prevent default browser behavior for arrow keys
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(event.key)) {
            event.preventDefault()
        }

        // Ignore if focus is on an input/textarea/select
        const tag = (event.target && event.target.tagName) || ''
        if (['INPUT', 'TEXTAREA', 'SELECT'].includes(tag)) return

        setPressedKeys((prevKeys) => {
            // Ignore if already pressed
            if (prevKeys.has(event.key)) {
                return prevKeys
            }

            const newPressedKeys = new Set(prevKeys)
            newPressedKeys.add(event.key)

            let linear = 0
            let angular = 0

            // Calculate velocity based on pressed keys
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
        // If space was released, don't stop (only stop on press)
        if (event.key === ' ' || event.key === 'Space') {
            return
        }

        setPressedKeys((prevKeys) => {
            const newPressedKeys = new Set(prevKeys)
            newPressedKeys.delete(event.key)

            // Recalculate velocity with remaining keys
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

    useEffect(() => {
        // Attach keyboard listeners when component mounts
        window.addEventListener('keydown', handleKeyDown)
        window.addEventListener('keyup', handleKeyUp)

        // Cleanup on unmount
        return () => {
            window.removeEventListener('keydown', handleKeyDown)
            window.removeEventListener('keyup', handleKeyUp)
            // Send stop command on unmount
            sendCmdVel(0, 0)
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [])

    // Stop when page loses focus
    useEffect(() => {
        const handleBlur = () => {
            sendCmdVel(0, 0)
            setPressedKeys(new Set())
            setIsActive(false)
        }

        window.addEventListener('blur', handleBlur)
        return () => window.removeEventListener('blur', handleBlur)
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [])

    const isConnected = status?.connected ?? true
    const isRunning = status?.running ?? true

    const applyCommand = (linear, angular) => {
        currentCmdRef.current = { linear, angular }
        sendCmdVel(linear, angular)
        setIsActive(linear !== 0 || angular !== 0)
    }

    const handlePressStart = (cmd) => () => applyCommand(cmd.linear, cmd.angular)
    const handlePressEnd = () => applyCommand(0, 0)

    return (
        <PageContainer
            title="Turtlesim Demo"
            description="Keyboard-controlled ROS 2 turtlesim demonstration"
        >
            <div className="space-y-6">
                {/* Live View */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-4 md:p-6 relative overflow-hidden">
                    <div className="flex items-center justify-between mb-4">
                        <h2 className="text-lg font-semibold text-white">Live View</h2>
                        {isVisLoading && (
                            <div className="text-xs text-gray-400">Loading visualization...</div>
                        )}
                        {visError && (
                            <div className="text-xs text-red-400">Failed to load view</div>
                        )}
                    </div>

                    <div
                        className="relative rounded-lg border border-gray-800 overflow-hidden bg-gray-950"
                        style={{ width: '100%', aspectRatio: '1 / 1', maxHeight: '70vh' }}
                    >
                        {(!visData?.url || visError) && (
                            <div className="flex items-center justify-center h-96 text-gray-400 text-sm">
                                Visualization unavailable
                            </div>
                        )}
                        {visData?.url && (
                            <iframe
                                title="Turtlesim Live View"
                                src={visData.url}
                                className="absolute inset-0 w-full h-full border-0"
                                style={{ background: '#0f172a' }}
                                allow="clipboard-read; clipboard-write"
                            />
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
                {/* Status Panel */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-6">
                    <div className="flex items-center justify-between mb-4">
                        <h2 className="text-lg font-semibold text-white">Status</h2>
                        <div className="flex items-center gap-4">
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

                    {/* Velocity Display */}
                    <div className="grid grid-cols-2 gap-4">
                        <div className="bg-gray-800/50 rounded-lg p-4">
                            <div className="text-gray-400 text-sm mb-1">Linear Velocity</div>
                            <div className="text-2xl font-bold text-white">
                                {velocity.linear.toFixed(2)} <span className="text-sm text-gray-400">m/s</span>
                            </div>
                        </div>
                        <div className="bg-gray-800/50 rounded-lg p-4">
                            <div className="text-gray-400 text-sm mb-1">Angular Velocity</div>
                            <div className="text-2xl font-bold text-white">
                                {velocity.angular.toFixed(2)} <span className="text-sm text-gray-400">rad/s</span>
                            </div>
                        </div>
                    </div>

                    {/* Active Indicator */}
                    {isActive && (
                        <div className="mt-4 flex items-center gap-2 text-green-400">
                            <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse" />
                            <span className="text-sm font-medium">Keyboard control active</span>
                        </div>
                    )}
                </div>

                {/* Controls Info */}
                <div className="bg-gray-900 border border-gray-800 rounded-xl p-6">
                    <h2 className="text-lg font-semibold text-white mb-4 flex items-center gap-2">
                        <Keyboard className="w-5 h-5 text-blue-400" />
                        Keyboard Controls
                    </h2>
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
                    </div>
                    {/* Fallback Control Pad */}
                    <div className="mt-6 grid grid-cols-3 gap-3 max-w-md">
                        <button
                            onMouseDown={handlePressStart({ linear: LINEAR_VEL, angular: 0 })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: LINEAR_VEL, angular: 0 })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 text-white py-2 px-3 rounded-lg transition-colors col-start-2"
                        >
                            Forward
                        </button>
                        <button
                            onMouseDown={handlePressStart({ linear: 0, angular: ANGULAR_VEL })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: 0, angular: ANGULAR_VEL })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 text-white py-2 px-3 rounded-lg transition-colors"
                        >
                            Rotate L
                        </button>
                        <button
                            onMouseDown={handlePressStart({ linear: 0, angular: -ANGULAR_VEL })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: 0, angular: -ANGULAR_VEL })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 text-white py-2 px-3 rounded-lg transition-colors col-start-3"
                        >
                            Rotate R
                        </button>
                        <button
                            onMouseDown={handlePressStart({ linear: -LINEAR_VEL, angular: 0 })}
                            onMouseUp={handlePressEnd}
                            onMouseLeave={handlePressEnd}
                            onTouchStart={handlePressStart({ linear: -LINEAR_VEL, angular: 0 })}
                            onTouchEnd={handlePressEnd}
                            className="bg-gray-800 hover:bg-gray-700 text-white py-2 px-3 rounded-lg transition-colors col-start-2"
                        >
                            Backward
                        </button>
                        <button
                            onClick={handlePressEnd}
                            className="bg-red-700 hover:bg-red-600 text-white py-2 px-3 rounded-lg transition-colors col-start-2"
                        >
                            Stop
                        </button>
                    </div>
                    <div className="mt-4 pt-4 border-t border-gray-800">
                        <div className="flex items-center gap-3 text-gray-300">
                            <kbd className="px-2 py-1 bg-gray-800 rounded text-sm font-mono">Space</kbd>
                            <span>Stop / Release keys to stop</span>
                        </div>
                    </div>
                </div>

                {/* Info */}
                <div className="bg-blue-900/20 border border-blue-800/50 rounded-xl p-4">
                    <p className="text-sm text-blue-300">
                        <strong>Note:</strong> This is a demonstration page. Press keys to control the turtle.
                        Click outside the live view iframe to ensure keyboard input is captured.
                    </p>
                </div>
            </div>
        </PageContainer>
    )
}

export default Turtlesim

