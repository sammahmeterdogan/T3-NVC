// frontend/src/pages/Navigation.jsx
import React, { useState } from 'react'
import { Trash2 } from 'lucide-react'
import toast from 'react-hot-toast'
import PageContainer from '../components/layout/PageContainer'
import { navigationAPI } from '../services/api'

const createMockWaypoint = () => {
  // Basit mock: -5 ile 5 arasında rastgele koordinatlar
  const x = Number((Math.random() * 10 - 5).toFixed(2))
  const y = Number((Math.random() * 10 - 5).toFixed(2))
  return { id: Date.now() + Math.random(), x, y }
}

const Navigation = () => {
  const [waypoints, setWaypoints] = useState([])
  const [loopMode, setLoopMode] = useState(false)
  const [isExecuting, setIsExecuting] = useState(false)

  const handleAddWaypoint = () => {
    const wp = createMockWaypoint()
    setWaypoints((prev) => [...prev, wp])
  }

  const handleDeleteWaypoint = (id) => {
    setWaypoints((prev) => prev.filter((wp) => wp.id !== id))
  }

  const handleClearAll = () => {
    setWaypoints([])
  }

  const handleExecuteMission = async () => {
    if (!waypoints.length) {
      toast.error('No waypoints defined')
      return
    }

    setIsExecuting(true)
    try {
      let iteration = 0
      do {
        iteration += 1
        // Her iterasyonda tüm waypoint'leri sırayla çalıştır
        /* eslint-disable no-await-in-loop */
        for (let i = 0; i < waypoints.length; i += 1) {
          const wp = waypoints[i]
          toast(`Going to Waypoint ${i + 1} (x=${wp.x}, y=${wp.y})...`, { id: `wp-${i}` })
          await navigationAPI.executeMission([wp])
        }
        /* eslint-enable no-await-in-loop */
      } while (loopMode && isExecuting)

      toast.success('Mission Complete')
    } catch (e) {
      console.error('[Navigation] Mission execution failed', e)
      toast.error('Mission failed')
    } finally {
      setIsExecuting(false)
    }
  }

  return (
    <PageContainer
      title="Navigation"
      description="Plan and execute multi-waypoint missions"
    >
      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6 h-[calc(100vh-200px)]">
        {/* Mission Planner Panel */}
        <div className="xl:col-span-1">
          <div className="bg-gray-900 border border-gray-800 rounded-xl h-full flex flex-col">
            {/* Header */}
            <div className="flex items-center justify-between px-4 py-3 border-b border-gray-800">
              <div>
                <h3 className="text-white font-semibold">Mission Planner</h3>
                <p className="text-xs text-gray-400">
                  Sequence multiple waypoints for autonomous navigation
                </p>
              </div>
              <label className="flex items-center gap-2 text-xs text-gray-300">
                <input
                  type="checkbox"
                  className="rounded border-gray-700 bg-gray-900 text-blue-500 focus:ring-blue-600"
                  checked={loopMode}
                  onChange={(e) => setLoopMode(e.target.checked)}
                  disabled={isExecuting}
                />
                <span>Loop Mode</span>
              </label>
            </div>

            {/* Waypoints List */}
            <div className="flex-1 overflow-y-auto px-4 py-3">
              {waypoints.length === 0 ? (
                <p className="text-xs text-gray-500">
                  No waypoints defined. Use &quot;Add Current Pose&quot; to create a mission.
                </p>
              ) : (
                <ul className="divide-y divide-gray-800">
                  {waypoints.map((wp, index) => (
                    <li
                      key={wp.id}
                      className="flex items-center justify-between py-2 text-xs"
                    >
                      <div className="flex flex-col">
                        <span className="text-gray-300 font-mono">
                          WP {index + 1}: x={wp.x}, y={wp.y}
                        </span>
                      </div>
                      <button
                        type="button"
                        onClick={() => handleDeleteWaypoint(wp.id)}
                        disabled={isExecuting}
                        className="p-1 text-gray-500 hover:text-red-400 rounded-lg hover:bg-gray-800 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                        title="Delete waypoint"
                      >
                        <Trash2 className="w-4 h-4" />
                      </button>
                    </li>
                  ))}
                </ul>
              )}
            </div>

            {/* Controls */}
            <div className="px-4 py-3 border-t border-gray-800 space-y-2">
              <div className="flex gap-2">
                <button
                  type="button"
                  onClick={handleAddWaypoint}
                  disabled={isExecuting}
                  className="flex-1 px-3 py-2 text-xs font-medium rounded-lg bg-gray-800 hover:bg-gray-700 text-gray-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  Add Current Pose
                </button>
                <button
                  type="button"
                  onClick={handleClearAll}
                  disabled={isExecuting || waypoints.length === 0}
                  className="px-3 py-2 text-xs font-medium rounded-lg bg-gray-800 hover:bg-gray-700 text-gray-300 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  Clear All
                </button>
              </div>
              <button
                type="button"
                onClick={handleExecuteMission}
                disabled={isExecuting || waypoints.length === 0}
                className="w-full px-3 py-2 text-xs font-semibold rounded-lg bg-green-600 hover:bg-green-700 text-white transition-colors disabled:bg-gray-700 disabled:cursor-not-allowed"
              >
                {isExecuting ? 'Executing Mission...' : 'EXECUTE MISSION'}
              </button>
            </div>
          </div>
        </div>

        {/* Placeholder columns to match mevcut layout (örneğin harita / kontroller) */}
        <div className="xl:col-span-2">
          <div className="h-full bg-gray-900 border border-gray-800 rounded-xl flex items-center justify-center">
            <p className="text-sm text-gray-500">
              Navigation map and controls can be integrated here.
            </p>
          </div>
        </div>
      </div>
    </PageContainer>
  )
}

export default Navigation


