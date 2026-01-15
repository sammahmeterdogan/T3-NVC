#!/usr/bin/env python3
"""
SO-101 Utility Functions
Based on ECE 4560 Assignment 8

This module provides utility functions for robot arm control including:
- Trajectory interpolation (move_to_pose)
- Position holding (hold_position)
- Pick and place sequences (pick_up_block, place_block)

These functions are designed to be generic and work with callback functions
rather than direct hardware access, making them compatible with both ROS
and Direct USB modes.
"""

import time
import math


def linear_interpolate(start_value, end_value, alpha):
    """
    Linear interpolation between two values.
    
    Args:
        start_value: Starting value
        end_value: Ending value
        alpha: Interpolation factor [0, 1]
    
    Returns:
        Interpolated value
    """
    return (1 - alpha) * start_value + alpha * end_value


def move_to_pose(starting_position, desired_position, duration, command_callback, 
                 read_callback=None, loop_rate_hz=50):
    """
    Move robot from starting position to desired position with smooth interpolation.
    
    This function generates a trajectory by linearly interpolating between
    starting and desired joint positions over the specified duration.
    
    Args:
        starting_position: Dict of joint names to starting angles (degrees)
        desired_position: Dict of joint names to target angles (degrees)
        duration: Time to complete motion (seconds)
        command_callback: Function(position_dict) to send commands
        read_callback: Optional function() that returns current position dict
        loop_rate_hz: Control loop frequency (default 50Hz)
    
    Example:
        def send_cmd(pos):
            bus.sync_write("Goal_Position", pos, normalize=True)
        
        move_to_pose(start, target, 2.0, send_cmd)
    """
    if duration <= 0:
        # Instant move
        command_callback(desired_position)
        return
    
    start_time = time.time()
    dt = 1.0 / loop_rate_hz
    
    # Ensure both dicts have same keys
    joint_names = set(starting_position.keys()) & set(desired_position.keys())
    
    while True:
        elapsed = time.time() - start_time
        
        if elapsed >= duration:
            # Send final position and exit
            command_callback(desired_position)
            break
        
        # Interpolation factor [0, 1] (clamped to prevent overshoot)
        alpha = min(elapsed / duration, 1.0)
        
        # Interpolate each joint
        interpolated_position = {}
        for joint in joint_names:
            p0 = starting_position[joint]
            pf = desired_position[joint]
            interpolated_position[joint] = linear_interpolate(p0, pf, alpha)
        
        # Send command
        command_callback(interpolated_position)
        
        # Optional: Read back current position for debugging
        if read_callback:
            current_pos = read_callback()
            # Could add tracking error monitoring here
        
        # Sleep to maintain loop rate
        time.sleep(dt)


def hold_position(position, hold_time, command_callback, loop_rate_hz=50):
    """
    Hold robot at a specific position for a given duration.
    
    Continuously sends position commands to maintain the pose against
    external disturbances or servo drift.
    
    Args:
        position: Dict of joint names to angles (degrees)
        hold_time: Duration to hold (seconds)
        command_callback: Function(position_dict) to send commands
        loop_rate_hz: Control loop frequency (default 50Hz)
    """
    start_time = time.time()
    dt = 1.0 / loop_rate_hz
    
    while time.time() - start_time < hold_time:
        command_callback(position)
        time.sleep(dt)


def pick_up_block(block_position, move_duration, ik_solver, pose_executor, 
                  raise_height=0.03, gripper_open=50, gripper_closed=5):
    """
    Execute pick-up sequence for a block at a given position.
    
    Sequence:
    1. Move above block with gripper open
    2. Descend to block level
    3. Close gripper
    4. Raise block
    
    Args:
        block_position: [x, y, z] position of block (meters)
        move_duration: Time for each move segment (seconds)
        ik_solver: Function(position) -> joint_dict that computes IK
        pose_executor: Function(joint_dict, duration) that executes motion
        raise_height: Height to raise above block (default 3cm)
        gripper_open: Gripper value for open (0-100, default 50)
        gripper_closed: Gripper value for closed (0-100, default 5)
    
    Returns:
        Final joint configuration (raised with gripper closed)
    
    Example:
        from so101_inverse_kinematics import get_inverse_kinematics
        
        def execute(joints, dur):
            move_to_pose(current_pos, joints, dur, send_command)
        
        final_config = pick_up_block([0.2, 0.0, 0.014], 2.0, 
                                     get_inverse_kinematics, execute)
    """
    # Step 1: Move above block with gripper open
    block_raised = block_position.copy()
    block_raised[2] += raise_height
    
    config_raised = ik_solver(block_raised)
    if config_raised is None:
        raise ValueError(f"Position {block_raised} is unreachable")
    
    config_raised['gripper'] = gripper_open
    pose_executor(config_raised, move_duration)
    
    # Step 2: Descend to block with gripper open
    config_at_block = ik_solver(block_position)
    if config_at_block is None:
        raise ValueError(f"Position {block_position} is unreachable")
    
    config_at_block['gripper'] = gripper_open
    pose_executor(config_at_block, 1.0)  # Slower descent
    
    # Step 3: Close gripper
    config_grip_closed = config_at_block.copy()
    config_grip_closed['gripper'] = gripper_closed
    pose_executor(config_grip_closed, 1.0)
    
    # Step 4: Raise with gripper closed
    config_raised['gripper'] = gripper_closed
    pose_executor(config_raised, 1.0)
    
    return config_raised


def place_block(target_position, move_duration, ik_solver, pose_executor,
                raise_height=0.03, gripper_open=50, gripper_closed=5):
    """
    Execute place sequence for a block at a target position.
    
    Sequence:
    1. Move above target with gripper closed
    2. Descend to target level
    3. Open gripper
    4. Raise arm
    
    Args:
        target_position: [x, y, z] position to place block (meters)
        move_duration: Time for each move segment (seconds)
        ik_solver: Function(position) -> joint_dict that computes IK
        pose_executor: Function(joint_dict, duration) that executes motion
        raise_height: Height to raise above target (default 3cm)
        gripper_open: Gripper value for open (0-100, default 50)
        gripper_closed: Gripper value for closed (0-100, default 5)
    
    Returns:
        Final joint configuration (raised with gripper open)
    """
    # Step 1: Move above target with gripper closed
    target_raised = target_position.copy()
    target_raised[2] += raise_height
    
    config_raised = ik_solver(target_raised)
    if config_raised is None:
        raise ValueError(f"Position {target_raised} is unreachable")
    
    config_raised['gripper'] = gripper_closed
    pose_executor(config_raised, move_duration)
    
    # Step 2: Descend to target with gripper closed
    config_at_target = ik_solver(target_position)
    if config_at_target is None:
        raise ValueError(f"Position {target_position} is unreachable")
    
    config_at_target['gripper'] = gripper_closed
    pose_executor(config_at_target, 1.0)
    
    # Step 3: Open gripper
    config_grip_open = config_at_target.copy()
    config_grip_open['gripper'] = gripper_open
    pose_executor(config_grip_open, 1.0)
    
    # Step 4: Raise with gripper open
    config_raised['gripper'] = gripper_open
    pose_executor(config_raised, 1.0)
    
    return config_raised


def pick_and_place_sequence(pick_position, place_position, move_duration,
                            ik_solver, pose_executor, home_position=None):
    """
    Complete pick-and-place sequence.
    
    Args:
        pick_position: [x, y, z] where to pick from (meters)
        place_position: [x, y, z] where to place (meters)
        move_duration: Time for each major move (seconds)
        ik_solver: IK solver function
        pose_executor: Motion execution function
        home_position: Optional [x, y, z] to return to after placing
    
    Returns:
        True on success, False on failure
    """
    try:
        # Pick up block
        print(f"Picking up block at {pick_position}...")
        pick_up_block(pick_position, move_duration, ik_solver, pose_executor)
        
        # Place block
        print(f"Placing block at {place_position}...")
        place_block(place_position, move_duration, ik_solver, pose_executor)
        
        # Return home if specified
        if home_position:
            print(f"Returning to home position {home_position}...")
            home_config = ik_solver(home_position)
            if home_config:
                home_config['gripper'] = 50
                pose_executor(home_config, move_duration)
        
        print("Pick-and-place sequence completed successfully!")
        return True
        
    except ValueError as e:
        print(f"Pick-and-place failed: {e}")
        return False


if __name__ == '__main__':
    # Test trajectory interpolation
    print("SO-101 Utility Functions Test\n")
    
    # Mock command callback
    commands_sent = []
    def mock_command(pos):
        commands_sent.append(pos.copy())
        print(f"  Command: {pos}")
    
    # Test move_to_pose
    print("Testing move_to_pose...")
    start = {'base': 0, 'shoulder': 0, 'elbow': 0}
    target = {'base': 90, 'shoulder': 45, 'elbow': -45}
    
    move_to_pose(start, target, 0.5, mock_command, loop_rate_hz=10)
    
    print(f"\nTotal commands sent: {len(commands_sent)}")
    print(f"First command: {commands_sent[0]}")
    print(f"Last command: {commands_sent[-1]}")
    
    # Verify interpolation
    assert abs(commands_sent[-1]['base'] - 90) < 0.1
    assert abs(commands_sent[-1]['shoulder'] - 45) < 0.1
    print("\n✓ Interpolation test passed!")
