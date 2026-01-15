#!/usr/bin/env python3
"""
SO-101 Inverse Kinematics Solver
Based on ECE 4560 Assignment 7 - Geometric Approach

This module implements inverse kinematics for the SO-ARM101 5-DOF robot arm.
The solver uses a geometric approach to compute joint angles for a given 
end-effector position and orientation.

Joint Configuration:
    - shoulder_pan (θ1): Base rotation (-180° to 180°)
    - shoulder_lift (θ2): Shoulder pitch (-90° to 90°)
    - elbow_flex (θ3): Elbow pitch (-135° to 135°)
    - wrist_flex (θ4): Wrist pitch (-90° to 90°)
    - wrist_roll (θ5): Wrist roll (-180° to 180°)
    - gripper: Gripper opening (0-100%)

DH Parameters (approximate for SO-ARM101):
    Link lengths in meters:
    - L1 = 0.061 (base to shoulder)
    - L2 = 0.105 (shoulder to elbow)
    - L3 = 0.095 (elbow to wrist)
    - L4 = 0.062 (wrist to end-effector)
"""

import numpy as np
import math


# Robot Physical Constants (in meters)
L1 = 0.061  # Base height to shoulder
L2 = 0.105  # Shoulder to elbow
L3 = 0.095  # Elbow to wrist
L4 = 0.062  # Wrist to end-effector
OFFSET_X = 0.019  # X-axis offset from base frame


# Joint Limits (in degrees)
JOINT_LIMITS = {
    'shoulder_pan': (-180, 180),
    'shoulder_lift': (-90, 90),
    'elbow_flex': (-135, 135),
    'wrist_flex': (-90, 90),
    'wrist_roll': (-180, 180),
    'gripper': (0, 100)
}


def clamp_joint(joint_name, angle):
    """Clamp joint angle to physical limits."""
    min_val, max_val = JOINT_LIMITS[joint_name]
    return max(min_val, min(max_val, angle))


def get_wrist_position(target_position, target_orientation=None):
    """
    Calculate the desired wrist center position from end-effector target.
    
    Assumes grasp-from-above orientation (tool Z-axis aligned with world Z-axis).
    
    Args:
        target_position: [x, y, z] in meters
        target_orientation: Optional orientation (not used in simplified version)
    
    Returns:
        wrist_position: [x, y, z] of wrist center
    """
    x_tool, y_tool, z_tool = target_position
    
    # For grasp-from-above: wrist is L4 meters above the tool point
    # (assuming tool points downward along -Z axis)
    x_wrist = x_tool
    y_wrist = y_tool
    z_wrist = z_tool + L4
    
    return np.array([x_wrist, y_wrist, z_wrist])


def solve_theta1(wrist_position):
    """
    Solve for θ1 (shoulder_pan) using projection onto ground plane.
    
    Args:
        wrist_position: [x, y, z] of wrist center
    
    Returns:
        theta1 in degrees
    """
    x_w, y_w, z_w = wrist_position
    
    # Account for base frame X offset
    x_adjusted = x_w - OFFSET_X
    
    # Project onto XY plane and compute angle
    theta1_rad = math.atan2(y_w, x_adjusted)
    theta1_deg = math.degrees(theta1_rad)
    
    return clamp_joint('shoulder_pan', theta1_deg)


def solve_theta2_theta3(wrist_position, theta1):
    """
    Solve for θ2 (shoulder_lift) and θ3 (elbow_flex) using planar geometry.
    
    Uses law of cosines on the triangle formed by shoulder-elbow-wrist.
    
    Args:
        wrist_position: [x, y, z] of wrist center
        theta1: Already-computed base rotation (degrees)
    
    Returns:
        (theta2, theta3) in degrees
    """
    x_w, y_w, z_w = wrist_position
    
    # Distance from base to wrist in XY plane
    x_adjusted = x_w - OFFSET_X
    r = math.sqrt(x_adjusted**2 + y_w**2)
    
    # Height from shoulder joint to wrist
    h = z_w - L1
    
    # Distance from shoulder to wrist (hypotenuse)
    d = math.sqrt(r**2 + h**2)
    
    # Check if target is reachable
    max_reach = L2 + L3
    min_reach = abs(L2 - L3)
    
    if d > max_reach or d < min_reach:
        # Unreachable - return None to signal error
        return None, None
    
    # Law of cosines to find elbow angle
    # cos(θ3) = (d² - L2² - L3²) / (2 * L2 * L3)
    cos_theta3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # Clamp to valid range
    
    theta3_rad = math.acos(cos_theta3)
    # Use negative angle for elbow-down configuration
    theta3_deg = -math.degrees(theta3_rad)
    
    # Solve for shoulder angle
    # Use law of sines and geometry
    alpha = math.atan2(h, r)
    beta = math.asin((L3 * math.sin(theta3_rad)) / d)
    
    theta2_rad = alpha - beta
    theta2_deg = math.degrees(theta2_rad)
    
    # Clamp to joint limits
    theta2_deg = clamp_joint('shoulder_lift', theta2_deg)
    theta3_deg = clamp_joint('elbow_flex', theta3_deg)
    
    return theta2_deg, theta3_deg


def solve_theta4(theta2, theta3):
    """
    Solve for θ4 (wrist_flex) to maintain horizontal gripper orientation.
    
    For grasp-from-above, the end-effector should point downward.
    This means: θ2 + θ3 + θ4 = 0 (to keep end-effector vertical)
    
    Args:
        theta2: Shoulder lift angle (degrees)
        theta3: Elbow flex angle (degrees)
    
    Returns:
        theta4 in degrees
    """
    # Keep gripper horizontal (pointing down)
    theta4_deg = -(theta2 + theta3)
    
    return clamp_joint('wrist_flex', theta4_deg)


def get_inverse_kinematics(target_position, target_orientation=None, gripper_value=None):
    """
    Main inverse kinematics solver for SO-ARM101.
    
    Uses geometric approach to compute joint configuration for a given
    end-effector position.
    
    Args:
        target_position: List [x, y, z] in meters
        target_orientation: Optional dict with 'pitch', 'roll', 'yaw' (not implemented)
        gripper_value: Optional gripper opening percentage (0-100)
    
    Returns:
        dict: Joint configuration or None if position is unreachable
        {
            'shoulder_pan': float,
            'shoulder_lift': float,
            'elbow_flex': float,
            'wrist_flex': float,
            'wrist_roll': float,
            'gripper': float
        }
    """
    # Input validation
    if not isinstance(target_position, (list, tuple, np.ndarray)) or len(target_position) != 3:
        return None
    
    try:
        # Step 1: Calculate wrist center position
        wrist_pos = get_wrist_position(target_position, target_orientation)
        
        # Step 2: Solve for θ1 (base rotation)
        theta1 = solve_theta1(wrist_pos)
        
        # Step 3: Solve for θ2 and θ3 (shoulder and elbow)
        theta2, theta3 = solve_theta2_theta3(wrist_pos, theta1)
        
        if theta2 is None or theta3 is None:
            # Position unreachable
            return None
        
        # Step 4: Solve for θ4 (wrist pitch to keep gripper horizontal)
        theta4 = solve_theta4(theta2, theta3)
        
        # Step 5: θ5 (wrist roll) defaults to 0 for now
        theta5 = 0.0
        
        # Step 6: Gripper value
        gripper = gripper_value if gripper_value is not None else 50.0
        gripper = clamp_joint('gripper', gripper)
        
        # Build joint configuration dictionary
        joint_config = {
            'shoulder_pan': theta1,
            'shoulder_lift': theta2,
            'elbow_flex': theta3,
            'wrist_flex': theta4,
            'wrist_roll': theta5,
            'gripper': gripper
        }
        
        return joint_config
        
    except (ValueError, ZeroDivisionError, ArithmeticError) as e:
        # Mathematical error - position likely unreachable
        print(f"[IK Solver] Error: {e}")
        return None


def check_workspace(target_position):
    """
    Check if a target position is within the robot's workspace.
    
    Args:
        target_position: [x, y, z] in meters
    
    Returns:
        bool: True if reachable, False otherwise
    """
    wrist_pos = get_wrist_position(target_position)
    x_w, y_w, z_w = wrist_pos
    
    # Distance in XY plane
    x_adjusted = x_w - OFFSET_X
    r = math.sqrt(x_adjusted**2 + y_w**2)
    
    # Height from shoulder
    h = z_w - L1
    
    # 3D distance from shoulder to wrist
    d = math.sqrt(r**2 + h**2)
    
    # Check against max/min reach
    max_reach = L2 + L3
    min_reach = abs(L2 - L3)
    
    return min_reach <= d <= max_reach


# Mapping to UI joint names (for convenience)
def convert_to_ui_format(joint_config):
    """
    Convert joint configuration from assignment format to UI format.
    
    Args:
        joint_config: Dict with assignment joint names
    
    Returns:
        Dict with UI joint names (base, shoulder, elbow, wristPitch, wristRoll, gripper)
    """
    if joint_config is None:
        return None
    
    return {
        'base': joint_config['shoulder_pan'],
        'shoulder': joint_config['shoulder_lift'],
        'elbow': joint_config['elbow_flex'],
        'wristPitch': joint_config['wrist_flex'],
        'wristRoll': joint_config['wrist_roll'],
        'gripper': joint_config['gripper']
    }


if __name__ == '__main__':
    # Test the IK solver
    print("SO-101 Inverse Kinematics Solver Test\n")
    
    test_positions = [
        [0.15, 0.0, 0.08],    # Forward (moderate reach)
        [0.12, 0.10, 0.06],   # Diagonal right
        [0.10, 0.0, 0.12],    # High and close
    ]
    
    for pos in test_positions:
        print(f"\nTarget Position: {pos}")
        
        # Check workspace
        if check_workspace(pos):
            print("  ✓ Within workspace")
        else:
            print("  ✗ Out of workspace")
            continue
        
        # Compute IK
        result = get_inverse_kinematics(pos)
        
        if result:
            print("  Joint Configuration:")
            for joint, angle in result.items():
                print(f"    {joint:15s}: {angle:7.2f}°")
            
            # Show UI format
            ui_format = convert_to_ui_format(result)
            print("\n  UI Format:")
            for joint, angle in ui_format.items():
                print(f"    {joint:15s}: {angle:7.2f}°")
        else:
            print("  ✗ IK solution failed")
