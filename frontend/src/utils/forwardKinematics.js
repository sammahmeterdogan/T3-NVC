/**
 * SO-ARM101 Forward Kinematics Solver
 * 
 * Computes end-effector position from joint angles.
 * Used for FK→IK mode transitions.
 * 
 * Units: Input angles in DEGREES, output positions in METERS
 */

import { ROBOT_CONFIG, degToRad } from './robotConfig'

const { L1, L2, L3, L4 } = ROBOT_CONFIG.links
const { x: OFFSET_X, z: OFFSET_Z } = ROBOT_CONFIG.baseOffset

/**
 * Compute end-effector position from joint angles.
 * 
 * @param {Object} joints - Joint configuration in UI format
 * @param {number} joints.base - Base rotation (degrees)
 * @param {number} joints.shoulder - Shoulder pitch (degrees)
 * @param {number} joints.elbow - Elbow pitch (degrees)
 * @param {number} joints.wristPitch - Wrist pitch (degrees)
 * @returns {Array<number>} End-effector position [x, y, z] in meters
 * 
 * @example
 * const position = computeFK({ base: 0, shoulder: -25, elbow: -102, wristPitch: -52 })
 * // Returns: [0.15, 0.0, 0.08] (approximately)
 */
export function computeFK(joints) {
    // Convert to radians
    const θ1 = degToRad(joints.base || 0)
    const θ2 = degToRad(joints.shoulder || 0)
    const θ3 = degToRad(joints.elbow || 0)
    const θ4 = degToRad(joints.wristPitch || 0)

    // Compute wrist position in cylindrical coordinates
    // r: radial distance from base axis
    // h: height from base
    const r = L2 * Math.cos(θ2) + L3 * Math.cos(θ2 + θ3)
    const h = L2 * Math.sin(θ2) + L3 * Math.sin(θ2 + θ3) + L1

    // Convert to Cartesian coordinates
    const x_wrist = r * Math.cos(θ1) + OFFSET_X
    const y_wrist = r * Math.sin(θ1)
    const z_wrist = h

    // End-effector is L4 below wrist (assuming downward gripper)
    // The wrist pitch θ4 affects the tool orientation, but for simplified FK
    // we assume the tool points straight down
    const x = x_wrist
    const y = y_wrist
    const z = z_wrist - L4

    return [x, y, z]
}

/**
 * Compute wrist center position from joint angles.
 * Useful for debugging and visualization.
 * 
 * @param {Object} joints - Joint configuration in UI format
 * @returns {Array<number>} Wrist position [x, y, z] in meters
 */
export function computeWristFK(joints) {
    const θ1 = degToRad(joints.base || 0)
    const θ2 = degToRad(joints.shoulder || 0)
    const θ3 = degToRad(joints.elbow || 0)

    const r = L2 * Math.cos(θ2) + L3 * Math.cos(θ2 + θ3)
    const h = L2 * Math.sin(θ2) + L3 * Math.sin(θ2 + θ3) + L1

    const x = r * Math.cos(θ1) + OFFSET_X
    const y = r * Math.sin(θ1)
    const z = h

    return [x, y, z]
}

export default computeFK
