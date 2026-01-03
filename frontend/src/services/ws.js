// frontend/src/services/ws.js
import { Client } from '@stomp/stompjs'
import SockJS from 'sockjs-client'

function resolveHttpUrl(input) {
    if (!input) return `${window.location.origin}/ws/robot`
    if (input.startsWith('/')) return `${window.location.origin}${input}`
    return input
}

// Connection states
const STATE = {
    DISCONNECTED: 'DISCONNECTED',
    CONNECTING: 'CONNECTING',
    CONNECTED: 'CONNECTED',
    ERROR: 'ERROR'
}

class WebSocketService {
    constructor() {
        this.client = null
        this.state = STATE.DISCONNECTED
        this.subscriptions = new Map()
        this.queuedSubscriptions = []
        this.reconnectAttempts = 0
        this.maxReconnectAttempts = 10
        this.baseReconnectDelay = 500
        this.maxReconnectDelay = 8000
        this.isDestroyed = false
        this.defaultUrl = (import.meta.env && import.meta.env.VITE_WS_URL) || '/ws/robot'
        this.connectPromise = null
        this.reconnectTimer = null
    }

    connect(url = this.defaultUrl) {
        if (this.isDestroyed) {
            return Promise.reject(new Error('WebSocketService destroyed'))
        }

        // Dedupe: if already connected or connecting, return existing promise
        if (this.state === STATE.CONNECTED) {
            console.log('[STOMP] Already connected')
            return Promise.resolve()
        }

        if (this.connectPromise) {
            console.log('[STOMP] Connection in progress, reusing promise')
            return this.connectPromise
        }

        const httpUrl = resolveHttpUrl(url)
        console.log(`[STOMP] CONNECTING to ${httpUrl}`)
        this.state = STATE.CONNECTING

        this.connectPromise = new Promise((resolve, reject) => {
            try {
                // Clean up old client if exists
                if (this.client) {
                    try {
                        this.client.deactivate()
                    } catch (e) {
                        console.warn('[STOMP] Failed to deactivate old client:', e)
                    }
                }

                this.client = new Client({
                    webSocketFactory: () => new SockJS(httpUrl),
                    debug: (str) => {
                        if (import.meta.env?.DEV) {
                            console.log('[STOMP]', str)
                        }
                    },
                    reconnectDelay: 0, // We handle reconnection manually
                    heartbeatIncoming: 4000,
                    heartbeatOutgoing: 4000,
                    
                    onConnect: (frame) => {
                        console.log(`[STOMP] CONNECTED to ${httpUrl}`)
                        this.state = STATE.CONNECTED
                        this.reconnectAttempts = 0
                        this.connectPromise = null
                        
                        // Process queued subscriptions
                        this.processQueuedSubscriptions()
                        
                        resolve()
                    },
                    
                    onStompError: (frame) => {
                        const msg = frame.headers?.message || 'STOMP protocol error'
                        console.error('[STOMP] ERROR:', msg, frame.body)
                        this.state = STATE.ERROR
                        this.connectPromise = null
                        reject(new Error(msg))
                    },
                    
                    onWebSocketClose: (evt) => {
                        console.warn('[STOMP] WebSocket closed', evt.reason || '')
                        
                        const wasConnected = this.state === STATE.CONNECTED
                        this.state = STATE.DISCONNECTED
                        this.connectPromise = null
                        
                        if (this.isDestroyed) {
                            console.log('[STOMP] Service destroyed, not reconnecting')
                            return
                        }
                        
                        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
                            console.error('[STOMP] Max reconnect attempts reached')
                            return
                        }
                        
                        // Exponential backoff with jitter
                        this.reconnectAttempts++
                        const baseDelay = Math.min(
                            this.baseReconnectDelay * Math.pow(2, this.reconnectAttempts - 1),
                            this.maxReconnectDelay
                        )
                        const jitter = Math.random() * 200
                        const delay = baseDelay + jitter
                        
                        console.log(`[STOMP] RETRY ${this.reconnectAttempts}/${this.maxReconnectAttempts} in ${Math.round(delay)}ms`)
                        
                        this.reconnectTimer = setTimeout(() => {
                            if (!this.isDestroyed) {
                                this.connect(url).catch(err => {
                                    console.error('[STOMP] Reconnect failed:', err.message)
                                })
                            }
                        }, delay)
                    },
                    
                    onWebSocketError: (evt) => {
                        console.error('[STOMP] WebSocket error', evt)
                    }
                })
                
                this.client.activate()
                
            } catch (err) {
                console.error('[STOMP] Connection setup failed:', err)
                this.state = STATE.ERROR
                this.connectPromise = null
                reject(err)
            }
        })

        return this.connectPromise
    }

    processQueuedSubscriptions() {
        if (this.queuedSubscriptions.length === 0) return
        
        console.log(`[STOMP] Processing ${this.queuedSubscriptions.length} queued subscriptions`)
        
        const queue = [...this.queuedSubscriptions]
        this.queuedSubscriptions = []
        
        queue.forEach(({ destination, callback }) => {
            try {
                this.subscribe(destination, callback)
            } catch (err) {
                console.error(`[STOMP] Failed to subscribe to ${destination}:`, err)
            }
        })
    }

    disconnect() {
        console.log('[STOMP] Disconnecting')
        
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer)
            this.reconnectTimer = null
        }
        
        // Clear queued subscriptions
        this.queuedSubscriptions = []
        
        // Unsubscribe all
        this.subscriptions.forEach((sub, dest) => {
            try {
                sub.unsubscribe()
            } catch (e) {
                console.warn(`[STOMP] Failed to unsubscribe from ${dest}:`, e)
            }
        })
        this.subscriptions.clear()
        
        // Deactivate client
        if (this.client) {
            try {
                this.client.deactivate()
            } catch (e) {
                console.warn('[STOMP] Failed to deactivate client:', e)
            }
        }
        
        this.state = STATE.DISCONNECTED
        this.connectPromise = null
    }

    destroy() {
        console.log('[STOMP] Destroying service')
        this.isDestroyed = true
        this.disconnect()
        this.client = null
    }

    isConnected() {
        return this.state === STATE.CONNECTED
    }

    getState() {
        return this.state
    }

    subscribe(destination, callback) {
        if (!callback || typeof callback !== 'function') {
            throw new Error('Callback must be a function')
        }

        // If not connected yet, queue the subscription
        if (this.state !== STATE.CONNECTED || !this.client) {
            console.log(`[STOMP] Not connected, queuing subscription to ${destination}`)
            this.queuedSubscriptions.push({ destination, callback })
            return () => {
                // Return cleanup function that removes from queue
                const idx = this.queuedSubscriptions.findIndex(
                    s => s.destination === destination && s.callback === callback
                )
                if (idx !== -1) {
                    this.queuedSubscriptions.splice(idx, 1)
                }
            }
        }

        // Already subscribed?
        if (this.subscriptions.has(destination)) {
            console.warn(`[STOMP] Already subscribed to ${destination}, skipping`)
            return () => this.unsubscribe(destination)
        }

        try {
            console.log(`[STOMP] Subscribing to ${destination}`)
            const sub = this.client.subscribe(destination, (message) => {
                try {
                    const body = message.body ? JSON.parse(message.body) : {}
                    callback(body)
                } catch (err) {
                    console.error(`[STOMP] Failed to parse message from ${destination}:`, err)
                    callback(message.body)
                }
            })
            
            this.subscriptions.set(destination, sub)
            
            // Return unsubscribe function
            return () => this.unsubscribe(destination)
            
        } catch (err) {
            console.error(`[STOMP] Subscribe to ${destination} failed:`, err)
            throw err
        }
    }

    unsubscribe(destination) {
        const sub = this.subscriptions.get(destination)
        if (sub) {
            try {
                console.log(`[STOMP] Unsubscribing from ${destination}`)
                sub.unsubscribe()
            } catch (err) {
                console.warn(`[STOMP] Failed to unsubscribe from ${destination}:`, err)
            }
            this.subscriptions.delete(destination)
        }
    }

    send(destination, body = {}) {
        if (this.state !== STATE.CONNECTED || !this.client) {
            console.warn(`[STOMP] Cannot send to ${destination}: not connected`)
            return false
        }
        
        try {
            this.client.publish({
                destination,
                body: JSON.stringify(body)
            })
            return true
        } catch (err) {
            console.error(`[STOMP] Failed to send to ${destination}:`, err)
            return false
        }
    }
}

export default WebSocketService
export const wsService = new WebSocketService()
