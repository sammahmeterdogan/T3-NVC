/**
 * SO-ARM101 Inverse Kinematics Solver
 * 
 * Client-side geometric IK solver for real-time interactive control.
 * Ported from Python backend (so101_inverse_kinematics.py).
 * 
 * Algorithm: Geometric approach using law of cosines
 * Units: Input positions in METERS, output angles in DEGREES
 */

import { ROBOT_CONFIG, clampJoint, degToRad, radToDeg, isInWorkspace } from './robotConfig'

const { L1, L2, L3, L4 } = ROBOT_CONFIG.links
const { x: OFFSET_X, z: OFFSET_Z } = ROBOT_CONFIG.baseOffset

/**
 * Calculate wrist center position from end-effector target.
 * Assumes grasp-from-above orientation (tool Z-axis aligned with world Z).
 * 
 * @param {Array<number>} targetPosition - [x, y, z] in meters
 * @returns {Array<number>} Wrist position [x, y, z]
 */
function getWristPosition(targetPosition) {
    const [x, y, z] = targetPosition

    // Wrist is L4 meters above the tool point (assuming downward gripper)
    return [x, y, z + L4]
}

/**
 * Solve for θ1 (base rotation / shoulder_pan).
 * 
 * @param {Array<number>} wristPosition - [x, y, z] of wrist center
 * @returns {number} theta1 in degrees
 */
function solveTheta1(wristPosition) {
    const [x_w, y_w] = wristPosition

    // Account for base frame X offset
    const x_adjusted = x_w - OFFSET_X

    // Project onto XY plane and compute angle
    const theta1_rad = Math.atan2(y_w, x_adjusted)
    const theta1_deg = radToDeg(theta1_rad)

    return clampJoint('shoulder_pan', theta1_deg)
}

/**
 * Solve for θ2 (shoulder_lift) and θ3 (elbow_flex) using planar geometry.
 * Uses law of cosines on the triangle formed by shoulder-elbow-wrist.
 * 
 * @param {Array<number>} wristPosition - [x, y, z] of wrist center
 * @param {number} theta1 - Base rotation in degrees
 * @returns {Object|null} {theta2, theta3} in degrees, or null if unreachable
 */
function solveTheta2Theta3(wristPosition, theta1) {
    const [x_w, y_w, z_w] = wristPosition

    // Distance from base to wrist in XY plane
    const x_adjusted = x_w - OFFSET_X
    const r = Math.sqrt(x_adjusted ** 2 + y_w ** 2)

    // Height from shoulder joint to wrist
    const h = z_w - L1

    // Distance from shoulder to wrist (hypotenuse)
    const d = Math.sqrt(r ** 2 + h ** 2)

    // Check if target is reachable
    const maxReach = L2 + L3
    const minReach = Math.abs(L2 - L3)

    if (d > maxReach || d < minReach) {
        return null // Unreachable
    }

    // Law of cosines to find elbow angle
    // cos(θ3) = (d² - L2² - L3²) / (2 * L2 * L3)
    let cos_theta3 = (d ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
    cos_theta3 = Math.max(-1.0, Math.min(1.0, cos_theta3)) // Clamp to valid range

    const theta3_rad = Math.acos(cos_theta3)
    // Use negative angle for elbow-down configuration
    const theta3_deg = -radToDeg(theta3_rad)

    // Solve for shoulder angle using law of sines and geometry
    const alpha = Math.atan2(h, r)
    const beta = Math.asin((L3 * Math.sin(theta3_rad)) / d)

    const theta2_rad = alpha - beta
    const theta2_deg = radToDeg(theta2_rad)

    // Clamp to joint limits
    const theta2_clamped = clampJoint('shoulder_lift', theta2_deg)
    const theta3_clamped = clampJoint('elbow_flex', theta3_deg)

    return { theta2: theta2_clamped, theta3: theta3_clamped }
}

/**
 * Solve for θ4 (wrist_flex) to maintain horizontal gripper orientation.
 * For grasp-from-above, the end-effector should point downward.
 * This means: θ2 + θ3 + θ4 = 0 (to keep end-effector vertical)
 * 
 * @param {number} theta2 - Shoulder lift angle in degrees
 * @param {number} theta3 - Elbow flex angle in degrees
 * @returns {number} theta4 in degrees
 */
function solveTheta4(theta2, theta3) {
    // Keep gripper horizontal (pointing down)
    const theta4_deg = -(theta2 + theta3)

    return clampJoint('wrist_flex', theta4_deg)
}

/**
 * Main inverse kinematics solver for SO-ARM101.
 * 
 * @param {Array<number>} targetPosition - [x, y, z] in meters
 * @param {Object} options - Optional parameters
 * @param {number} options.wristRoll - Desired wrist roll angle (default: 0)
 * @param {number} options.gripper - Desired gripper opening (default: 50)
 * @returns {Object|null} Joint configuration in UI format, or null if unreachable
 * 
 * @example
 * const result = solveIK([0.15, 0.0, 0.08])
 * // Returns: { base: 0, shoulder: -25.3, elbow: -102.1, wristPitch: -52.6, wristRoll: 0, gripper: 50 }
 * 
 * const unreachable = solveIK([1.0, 0.0, 0.0])
 * // Returns: null
 */
export function solveIK(targetPosition, options = {}) {
    // Input validation
    if (!Array.isArray(targetPosition) || targetPosition.length !== 3) {
        console.error('[IK] Invalid target position:', targetPosition)
        return null
    }

    const [x, y, z] = targetPosition

    // Quick workspace check
    if (!isInWorkspace(targetPosition)) {
        console.warn('[IK] Target out of workspace:', targetPosition)
        return null
    }

    try {
        // Step 1: Calculate wrist center position
        const wristPos = getWristPosition(targetPosition)

        // Step 2: Solve for θ1 (base rotation)
        const theta1 = solveTheta1(wristPos)

        // Step 3: Solve for θ2 and θ3 (shoulder and elbow)
        const result23 = solveTheta2Theta3(wristPos, theta1)

        if (!result23) {
            console.warn('[IK] No solution for shoulder/elbow:', targetPosition)
            return null
        }

        const { theta2, theta3 } = result23

        // Step 4: Solve for θ4 (wrist pitch to keep gripper horizontal)
        const theta4 = solveTheta4(theta2, theta3)

        // Step 5: θ5 (wrist roll) from options or default
        const theta5 = options.wristRoll !== undefined ? options.wristRoll : 0

        // Step 6: Gripper value from options or default
        const gripper = options.gripper !== undefined ? options.gripper : 50

        // Build joint configuration in UI format
        const jointConfig = {
            base: theta1,
            shoulder: theta2,
            elbow: theta3,
            wristPitch: theta4,
            wristRoll: clampJoint('wrist_roll', theta5),
            gripper: clampJoint('gripper', gripper),
        }

        return jointConfig

    } catch (error) {
        console.error('[IK] Solver error:', error)
        return null
    }
}

/**
 * Batch solve IK for multiple targets (useful for trajectory planning).
 * 
 * @param {Array<Array<number>>} targets - Array of [x, y, z] positions
 * @param {Object} options - Options passed to solveIK
 * @returns {Array<Object|null>} Array of joint configurations
 */
export function solveIKBatch(targets, options = {}) {
    return targets.map(target => solveIK(target, options))
}

export default solveIK
