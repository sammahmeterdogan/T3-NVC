import React, { useEffect, useRef, useState } from 'react'
import { ChevronUp, ChevronDown, TerminalSquare } from 'lucide-react'

// Basit log simülasyonu için kullanılacak seviyeler ve mesajlar
const LEVELS = ['INFO', 'WARN', 'ERROR']
const MESSAGES = [
  'ROS2 Node Active',
  'Battery at 80%',
  'WebSocket heartbeat OK',
  'SLAM mapping idle',
  'Navigation stack standing by',
  'ROSBridge connection stable',
  'NoVNC session active',
]

const getRandomItem = (arr) => arr[Math.floor(Math.random() * arr.length)]

const formatTimestamp = () => {
  const now = new Date()
  return now.toISOString().split('T')[1].replace('Z', '')
}

/**
 * LiveTerminal - Global sistem logları için alt kısımda sabit terminal paneli
 *
 * - Varsayılan olarak minimize başlar (sadece header bar görünür)
 * - Panel açıldığında son loglara otomatik scroll eder
 * - Şu anda örnek log üretiyor; gerçek zamanlı kaynağa ileride bağlanabilir
 */
const LiveTerminal = () => {
  const [isOpen, setIsOpen] = useState(false)
  const [logs, setLogs] = useState([])
  const containerRef = useRef(null)

  // Örnek log üretimi
  useEffect(() => {
    const interval = setInterval(() => {
      setLogs((prev) => {
        const level = getRandomItem(LEVELS)
        const message = getRandomItem(MESSAGES)
        const timestamp = formatTimestamp()
        const entry = `[${timestamp}] [${level}] ${message}`
        // Listeyi çok büyütmemek için son 200 entry'i tut
        const next = [...prev, entry]
        if (next.length > 200) {
          return next.slice(next.length - 200)
        }
        return next
      })
    }, 3000)

    return () => clearInterval(interval)
  }, [])

  // Açıkken yeni log geldiğinde otomatik olarak en alta scroll et
  useEffect(() => {
    if (!isOpen) return
    if (!containerRef.current) return

    containerRef.current.scrollTop = containerRef.current.scrollHeight
  }, [logs, isOpen])

  const toggleOpen = () => setIsOpen((prev) => !prev)

  return (
    <div className="fixed bottom-0 left-0 w-full z-50 pointer-events-none">
      <div className="max-w-[1920px] mx-auto px-4 pb-4">
        <div className="pointer-events-auto bg-gray-950 bg-opacity-95 border border-gray-800 rounded-t-xl shadow-lg shadow-black/40 overflow-hidden">
          {/* Header Bar */}
          <div className="flex items-center justify-between px-3 py-1.5 border-b border-gray-800">
            <div className="flex items-center gap-2">
              <TerminalSquare className="w-3 h-3 text-green-500" />
              <span className="text-[10px] font-mono tracking-[0.2em] text-green-400">
                SYSTEM TERMINAL
              </span>
            </div>
            <button
              type="button"
              onClick={toggleOpen}
              className="flex items-center gap-1 text-[10px] text-gray-400 hover:text-green-400 transition-colors"
            >
              <span>{isOpen ? 'Minimize' : 'Expand'}</span>
              {isOpen ? (
                <ChevronDown className="w-3 h-3" />
              ) : (
                <ChevronUp className="w-3 h-3" />
              )}
            </button>
          </div>

          {/* Content Area */}
          {isOpen && (
            <div
              ref={containerRef}
              className="h-48 overflow-y-auto bg-black bg-opacity-60 px-3 py-2 font-mono text-[11px] text-green-400"
            >
              {logs.length === 0 ? (
                <div className="text-gray-500">[BOOT] Waiting for system logs...</div>
              ) : (
                logs.map((line, idx) => (
                  <div key={`${line}-${idx}`} className="whitespace-pre">
                    {line}
                  </div>
                ))
              )}
            </div>
          )}
        </div>
      </div>
    </div>
  )
}

export default LiveTerminal


