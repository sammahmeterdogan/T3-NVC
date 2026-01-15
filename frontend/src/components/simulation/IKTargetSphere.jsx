/**
 * Interactive IK Target Sphere Component with TransformControls
 * 
 * Draggable 3D sphere with XYZ gizmo arrows for IK target control.
 * Uses @react-three/drei TransformControls for proper transform gizmo.
 * 
 * PIPELINE: Drag → onDrag(position) → handleIKTargetDrag → solveIK → joints → command
 */

import React, { useRef, useEffect, useState, useCallback } from 'react'
import { TransformControls, Line } from '@react-three/drei'
import { useThree, useFrame } from '@react-three/fiber'
import * as THREE from 'three'

const IKTargetSphere = ({ position, onDrag, enabled, isReachable = true }) => {
    const meshRef = useRef()
    const transformRef = useRef()
    const { gl, camera } = useThree()
    const [isDragging, setIsDragging] = useState(false)
    const [mounted, setMounted] = useState(false)

    // Force re-render after mount to ensure meshRef is available
    useEffect(() => {
        setMounted(true)
    }, [])

    // Sync position from props when not dragging
    useEffect(() => {
        if (meshRef.current && !isDragging) {
            meshRef.current.position.set(position[0], position[1], position[2])
        }
    }, [position, isDragging])

    // Handle transform changes - THIS IS THE CRITICAL WIRING
    useEffect(() => {
        if (!transformRef.current || !meshRef.current) return

        const controls = transformRef.current

        const handleDragStart = (event) => {
            if (event.value) {
                setIsDragging(true)
                gl.domElement.style.cursor = 'grabbing'
                console.log('[IK] IK_DRAG_START')
            } else {
                setIsDragging(false)
                gl.domElement.style.cursor = 'auto'
                console.log('[IK] IK_DRAG_END')
            }
        }

        const handleDrag = () => {
            if (meshRef.current && onDrag) {
                const pos = meshRef.current.position
                // Log for debugging - this proves drag is calling the handler
                console.log(`[IK] IK_DRAG world={x:${pos.x.toFixed(4)}, y:${pos.y.toFixed(4)}, z:${pos.z.toFixed(4)}}`)
                // Call the parent's drag handler - THIS IS THE KEY CONNECTION
                onDrag([pos.x, pos.y, pos.z])
            }
        }

        controls.addEventListener('dragging-changed', handleDragStart)
        controls.addEventListener('objectChange', handleDrag)

        return () => {
            controls.removeEventListener('dragging-changed', handleDragStart)
            controls.removeEventListener('objectChange', handleDrag)
        }
    }, [onDrag, gl, mounted])

    // Determine sphere color based on state
    const getColor = () => {
        if (!enabled) return '#666666'
        if (isDragging) return '#3b82f6' // Blue while dragging
        if (!isReachable) return '#ef4444' // Red when unreachable
        return '#22c55e' // Green when reachable
    }

    const getEmissive = () => {
        if (!enabled) return '#000000'
        if (isDragging) return '#3b82f6'
        if (!isReachable) return '#ef4444'
        return '#22c55e'
    }

    if (!enabled) return null

    return (
        <group>
            {/* Target Sphere */}
            <mesh ref={meshRef} position={position}>
                <sphereGeometry args={[0.02, 32, 32]} />
                <meshStandardMaterial
                    color={getColor()}
                    emissive={getEmissive()}
                    emissiveIntensity={isDragging ? 0.8 : 0.5}
                    roughness={0.3}
                    metalness={0.5}
                    transparent
                    opacity={0.9}
                />
            </mesh>

            {/* Transform Controls (XYZ arrows gizmo) - Always render when mounted */}
            {mounted && meshRef.current && (
                <TransformControls
                    ref={transformRef}
                    object={meshRef.current}
                    mode="translate"
                    size={0.7}
                    showX={true}
                    showY={true}
                    showZ={true}
                    space="world"
                />
            )}

            {/* Vertical line to ground (visual aid) */}
            <Line
                points={[
                    [position[0], position[1], position[2]],
                    [position[0], position[1], 0]
                ]}
                color={isReachable ? '#22c55e' : '#ef4444'}
                lineWidth={2}
                opacity={0.6}
                transparent
                dashed
                dashSize={0.01}
                gapSize={0.01}
            />

            {/* Workspace boundary ring on ground */}
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0.06, 0.001, 0]}>
                <ringGeometry args={[0.20, 0.205, 64]} />
                <meshBasicMaterial
                    color={isReachable ? '#22c55e' : '#ef4444'}
                    transparent
                    opacity={0.4}
                    side={2}
                />
            </mesh>
        </group>
    )
}

export default IKTargetSphere
