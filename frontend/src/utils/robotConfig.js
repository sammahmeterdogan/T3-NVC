/**
 * SO-ARM101 Robot Configuration
 * 
 * Authoritative source for robot geometry and joint limits.
 * All measurements in METERS, all angles in DEGREES.
 * 
 * Source: ECE 4560 Assignment 6/7 - MuJoCo transformation matrices
 */

// ============================================================================
// KINEMATIC PARAMETERS (meters)
// ============================================================================

export const ROBOT_CONFIG = {
    // Link lengths derived from transformation matrices
    links: {
        L1: 0.0692345,  // Base height to shoulder axis (World→J1: z=0.0388353 + J1→J2: z=0.0303992)
        L2: 0.11599,    // Shoulder to elbow (J2→J3: straight-line distance)
        L3: 0.1349,     // Elbow to wrist (J3→J4: x-offset)
        L4: 0.1645,     // Wrist to end-effector/tool tip (J4→J5: 0.0611 + J5→Tool: 0.1034)
    },

    // Base frame offsets from world origin
    baseOffset: {
        x: 0.0624,      // Horizontal offset
        y: 0.0,
        z: 0.0388353,   // Vertical offset
    },

    // Workspace constraints (meters)
    workspace: {
        maxReach: 0.415,  // L2 + L3 + L4 (approximate)
        minReach: 0.01,   // Near singularity zone
        minZ: 0.01,       // Ground clearance
    },

    // Joint limits (degrees) - STS3215 servo physical range
    jointLimits: {
        shoulder_pan: { min: -150, max: 150 },   // Base rotation
        shoulder_lift: { min: -150, max: 150 },  // Shoulder pitch
        elbow_flex: { min: -150, max: 150 },     // Elbow pitch
        wrist_flex: { min: -150, max: 150 },     // Wrist pitch
        wrist_roll: { min: -150, max: 150 },     // Wrist roll
        gripper: { min: 0, max: 100 },           // Gripper percentage
    },
}

// ============================================================================
// JOINT NAME MAPPING
// ============================================================================

/**
 * Canonical joint name mapping across the system.
 * 
 * UI State    → Assignment/URDF → Servo ID
 * ----------    ---------------   --------
 * base        → shoulder_pan    → 1
 * shoulder    → shoulder_lift   → 2
 * elbow       → elbow_flex      → 3
 * wristPitch  → wrist_flex      → 4
 * wristRoll   → wrist_roll      → 5
 * gripper     → gripper         → 6
 */
export const JOINT_MAPPING = {
    // UI → Assignment format
    toAssignment: {
        base: 'shoulder_pan',
        shoulder: 'shoulder_lift',
        elbow: 'elbow_flex',
        wristPitch: 'wrist_flex',
        wristRoll: 'wrist_roll',
        gripper: 'gripper',
    },

    // Assignment → UI format
    toUI: {
        shoulder_pan: 'base',
        shoulder_lift: 'shoulder',
        elbow_flex: 'elbow',
        wrist_flex: 'wristPitch',
        wrist_roll: 'wristRoll',
        gripper: 'gripper',
    },

    // UI → Servo ID
    toServoID: {
        base: 1,
        shoulder: 2,
        elbow: 3,
        wristPitch: 4,
        wristRoll: 5,
        gripper: 6,
    },
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Clamp a joint angle to its physical limits.
 * @param {string} jointName - Assignment format joint name
 * @param {number} angle - Angle in degrees
 * @returns {number} Clamped angle
 */
export function clampJoint(jointName, angle) {
    const limits = ROBOT_CONFIG.jointLimits[jointName]
    if (!limits) {
        console.warn(`Unknown joint: ${jointName}`)
        return angle
    }
    return Math.max(limits.min, Math.min(limits.max, angle))
}

/**
 * Convert degrees to radians.
 */
export function degToRad(degrees) {
    return degrees * (Math.PI / 180)
}

/**
 * Convert radians to degrees.
 */
export function radToDeg(radians) {
    return radians * (180 / Math.PI)
}

/**
 * Check if a position is within the robot's workspace.
 * @param {Array<number>} position - [x, y, z] in meters
 * @returns {boolean} True if reachable
 */
export function isInWorkspace(position) {
    const [x, y, z] = position

    // Check Z bounds
    if (z < ROBOT_CONFIG.workspace.minZ) {
        return false
    }

    // Calculate distance from base
    const { L1, L2, L3, L4 } = ROBOT_CONFIG.links
    const { x: offsetX, z: offsetZ } = ROBOT_CONFIG.baseOffset

    // Wrist position (target is end-effector, wrist is L4 above it)
    const wristZ = z + L4

    // Distance in XY plane from base
    const r = Math.sqrt((x - offsetX) ** 2 + y ** 2)

    // Height from shoulder
    const h = wristZ - L1

    // 3D distance from shoulder to wrist
    const d = Math.sqrt(r ** 2 + h ** 2)

    // Check against max/min reach
    const maxReach = L2 + L3
    const minReach = Math.abs(L2 - L3)

    return d >= minReach && d <= maxReach
}

export default ROBOT_CONFIG
