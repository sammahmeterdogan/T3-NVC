import React, { useState, useEffect, useCallback, useRef } from 'react'
import { motion } from 'framer-motion'
import { X, RefreshCw } from 'lucide-react'

/**
 * ArmControls - Bambot-style Joint Control Panel
 * Refactored to be a CONTROLLED COMPONENT.
 * It now receives `joints` from parent and bubbles changes up immediately.
 */
const ArmControls = ({
    enabled = true,
    disabled = false,
    joints, // NOW CONTROLLED BY PARENT
    onJointChange = () => { }
}) => {
    const isEnabled = enabled && !disabled

    // Joint Configuration mapping (Bambot mapping)
    const jointConfig = [
        { id: 'base', label: 'Rotation', keys: ['Q', '1'], min: -180, max: 180, step: 5 },
        { id: 'shoulder', label: 'Pitch', keys: ['W', '2'], min: -90, max: 90, step: 5 },
        { id: 'elbow', label: 'Elbow', keys: ['E', '3'], min: -135, max: 135, step: 5 },
        { id: 'wristPitch', label: 'Wrist P', keys: ['R', '4'], min: -90, max: 90, step: 5 },
        { id: 'wristRoll', label: 'Wrist R', keys: ['T', '5'], min: -180, max: 180, step: 5 },
        { id: 'gripper', label: 'Gripper', keys: ['Y', '6'], min: 0, max: 100, step: 10, unit: '%' }
    ]

    // Keyboard Listener
    useEffect(() => {
        const handleKeyDown = (e) => {
            if (!isEnabled || e.repeat) return
            const key = e.key.toUpperCase()

            let updated = false
            const newJoints = { ...joints }

            jointConfig.forEach(config => {
                const [downKey, upKey] = config.keys

                if (key === downKey) {
                    newJoints[config.id] = Math.max(config.min, newJoints[config.id] - config.step)
                    updated = true
                } else if (key === upKey) {
                    newJoints[config.id] = Math.min(config.max, newJoints[config.id] + config.step)
                    updated = true
                }
            })

            if (updated) {
                onJointChange(newJoints)
            }
        }

        window.addEventListener('keydown', handleKeyDown)
        return () => window.removeEventListener('keydown', handleKeyDown)
    }, [isEnabled, joints, onJointChange])

    // Slider Handler
    const handleSliderChange = (id, value) => {
        const newJoints = { ...joints, [id]: parseFloat(value) }
        onJointChange(newJoints)
    }

    // Compound Movements (Presets)
    const applyPreset = (preset) => {
        let newJoints = { ...joints }
        switch (preset) {
            case 'HOME':
                newJoints = { base: 0, shoulder: 0, elbow: 0, wristPitch: 0, wristRoll: 0, gripper: 50 };
                break;
            case 'READY':
                newJoints = { base: 0, shoulder: -45, elbow: 90, wristPitch: -45, wristRoll: 0, gripper: 50 };
                break;
            case 'GRAB':
                newJoints = { ...joints, gripper: 100 }; // Open
                break;
            case 'DROP':
                newJoints = { ...joints, gripper: 0 }; // Close
                break;
            default:
                return;
        }
        onJointChange(newJoints)
    }

    return (
        <div className="w-full bg-gray-900/95 backdrop-blur-sm border border-gray-700 rounded-xl shadow-2xl overflow-hidden font-mono text-gray-300">
            {/* Header */}
            <div className="flex items-center justify-between px-3 py-2 bg-gray-800/50 border-b border-gray-700">
                <div className="flex items-center gap-2">
                    <div className={`w-2 h-2 rounded-full ${isEnabled ? 'bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.6)]' : 'bg-red-500'}`} />
                    <span className="text-xs font-bold text-white tracking-wider">JOINT CONTROLS</span>
                </div>
                <button className="text-gray-500 hover:text-white transition-colors">
                    <X size={14} />
                </button>
            </div>

            {/* Joint Table */}
            <div className="p-3 space-y-3">
                {jointConfig.map(j => (
                    <div key={j.id} className="grid grid-cols-[80px_60px_1fr] items-center gap-2 text-[11px]">
                        {/* Name */}
                        <div className="font-semibold text-gray-400 truncate">{j.label}</div>

                        {/* Value */}
                        <div className="text-right font-mono text-cyan-400 bg-gray-900/50 rounded px-1">
                            {joints[j.id]?.toFixed(1)}{j.unit || '°'}
                        </div>

                        {/* Control: Key - Slider - Key */}
                        <div className="flex items-center gap-2">
                            <span className="w-5 h-5 flex items-center justify-center bg-gray-800 rounded border border-gray-700 text-[9px] text-gray-500 font-bold select-none">
                                {j.keys[0]}
                            </span>

                            <input
                                type="range"
                                min={j.min}
                                max={j.max}
                                step={1}
                                value={joints[j.id]}
                                onChange={(e) => handleSliderChange(j.id, e.target.value)}
                                disabled={!isEnabled}
                                className="flex-1 h-1 bg-gray-700 rounded-full appearance-none cursor-pointer [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:w-3 [&::-webkit-slider-thumb]:h-3 [&::-webkit-slider-thumb]:bg-blue-500 [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:shadow-lg hover:[&::-webkit-slider-thumb]:bg-blue-400 transition-all disabled:opacity-50"
                            />

                            <span className="w-5 h-5 flex items-center justify-center bg-gray-800 rounded border border-gray-700 text-[9px] text-gray-500 font-bold select-none">
                                {j.keys[1]}
                            </span>
                        </div>
                    </div>
                ))}
            </div>

            {/* Footer: Compound Movements */}
            <div className="px-3 py-2 bg-gray-800/30 border-t border-gray-700">
                <div className="grid grid-cols-2 gap-2">
                    <button
                        onClick={() => applyPreset('HOME')}
                        disabled={!isEnabled}
                        className="px-2 py-1.5 bg-gray-800 hover:bg-gray-700 border border-gray-600 rounded text-[10px] text-white transition-all active:scale-95 disabled:opacity-50 flex items-center justify-center gap-1"
                    >
                        <RefreshCw size={10} />
                        Home
                    </button>
                    <button
                        onClick={() => applyPreset('READY')}
                        disabled={!isEnabled}
                        className="px-2 py-1.5 bg-gray-800 hover:bg-gray-700 border border-gray-600 rounded text-[10px] text-white transition-all active:scale-95 disabled:opacity-50"
                    >
                        Ready Pose
                    </button>
                    <button
                        onClick={() => applyPreset('GRAB')}
                        disabled={!isEnabled}
                        className="px-2 py-1.5 bg-blue-900/30 hover:bg-blue-900/50 border border-blue-800/50 rounded text-[10px] text-blue-200 transition-all active:scale-95 disabled:opacity-50"
                    >
                        Open Grip
                    </button>
                    <button
                        onClick={() => applyPreset('DROP')}
                        disabled={!isEnabled}
                        className="px-2 py-1.5 bg-blue-900/30 hover:bg-blue-900/50 border border-blue-800/50 rounded text-[10px] text-blue-200 transition-all active:scale-95 disabled:opacity-50"
                    >
                        Close Grip
                    </button>
                </div>
            </div>
        </div>
    )
}

export default ArmControls
