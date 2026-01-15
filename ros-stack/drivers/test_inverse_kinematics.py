#!/usr/bin/env python3
"""
Unit tests for SO-101 Inverse Kinematics Solver

Run with: python -m pytest test_inverse_kinematics.py -v
Or: python test_inverse_kinematics.py
"""

import sys
import math
import numpy as np

# Import the IK solver
from so101_inverse_kinematics import (
    get_inverse_kinematics,
    check_workspace,
    convert_to_ui_format,
    clamp_joint,
    JOINT_LIMITS
)


def test_workspace_validation():
    """Test that workspace validation correctly identifies reachable positions."""
    print("\n=== Test: Workspace Validation ===")
    
    # Clearly reachable position (moderate reach)
    assert check_workspace([0.15, 0.0, 0.08]) == True, "Should be reachable"
    print("✓ Moderate reach position validated")
    
    # Position too far away
    assert check_workspace([0.5, 0.5, 0.5]) == False, "Should be unreachable (too far)"
    print("✓ Far position correctly rejected")
    
    # Position too close (inside minimum reach)
    assert check_workspace([0.01, 0.0, 0.05]) == False, "Should be unreachable (too close)"
    print("✓ Close position correctly rejected")
    
    print("PASSED: Workspace validation")


def test_joint_limits():
    """Test that joint angles are properly clamped to limits."""
    print("\n=== Test: Joint Limit Clamping ===")
    
    # Test shoulder_pan limits
    assert clamp_joint('shoulder_pan', 200) == 180, "Should clamp to max"
    assert clamp_joint('shoulder_pan', -200) == -180, "Should clamp to min"
    print("✓ shoulder_pan clamping works")
    
    # Test gripper limits
    assert clamp_joint('gripper', 150) == 100, "Gripper should clamp to 100"
    assert clamp_joint('gripper', -10) == 0, "Gripper should clamp to 0"
    print("✓ gripper clamping works")
    
    print("PASSED: Joint limit clamping")


def test_ik_known_positions():
    """Test IK solver with known good positions."""
    print("\n=== Test: Known Position Solutions ===")
    
    test_cases = [
        {
            'name': 'Forward center',
            'position': [0.15, 0.0, 0.08],
            'expected_approx': {
                'shoulder_pan': 0.0,  # Base should be centered
            }
        },
        {
            'name': 'Diagonal right',
            'position': [0.12, 0.10, 0.06],
            'expected_approx': {
                'shoulder_pan': 45.0,  # Should rotate ~45° to right
            }
        },
    ]
    
    for case in test_cases:
        pos = case['position']
        result = get_inverse_kinematics(pos)
        
        assert result is not None, f"Failed to find solution for {case['name']}"
        print(f"✓ {case['name']}: Solution found")
        
        # Check expected values within tolerance
        for joint, expected_val in case.get('expected_approx', {}).items():
            actual_val = result[joint]
            tolerance = 5.0  # degrees
            assert abs(actual_val - expected_val) < tolerance, \
                f"{joint} expected ~{expected_val}°, got {actual_val}°"
            print(f"  {joint}: {actual_val:.2f}° (expected ~{expected_val}°)")
    
    print("PASSED: Known position solutions")


def test_ik_returns_none_for_unreachable():
    """Test that IK returns None for unreachable positions."""
    print("\n=== Test: Unreachable Position Handling ===")
    
    unreachable_positions = [
        [1.0, 0.0, 0.0],  # Too far
        [0.0, 1.0, 0.0],  # Too far
        [0.01, 0.0, 0.01],  # Too close
    ]
    
    for pos in unreachable_positions:
        result = get_inverse_kinematics(pos)
        assert result is None, f"Should return None for unreachable position {pos}"
        print(f"✓ Correctly rejected {pos}")
    
    print("PASSED: Unreachable position handling")


def test_ui_format_conversion():
    """Test conversion to UI joint format."""
    print("\n=== Test: UI Format Conversion ===")
    
    assignment_format = {
        'shoulder_pan': 45.0,
        'shoulder_lift': -30.0,
        'elbow_flex': 60.0,
        'wrist_flex': -15.0,
        'wrist_roll': 0.0,
        'gripper': 75.0
    }
    
    ui_format = convert_to_ui_format(assignment_format)
    
    assert ui_format['base'] == 45.0, "shoulder_pan -> base"
    assert ui_format['shoulder'] == -30.0, "shoulder_lift -> shoulder"
    assert ui_format['elbow'] == 60.0, "elbow_flex -> elbow"
    assert ui_format['wristPitch'] == -15.0, "wrist_flex -> wristPitch"
    assert ui_format['wristRoll'] == 0.0, "wrist_roll -> wristRoll"
    assert ui_format['gripper'] == 75.0, "gripper -> gripper"
    
    print("✓ All joint names converted correctly")
    print("PASSED: UI format conversion")


def test_gripper_propagation():
    """Test that gripper value is correctly propagated through IK."""
    print("\n=== Test: Gripper Value Propagation ===")
    
    pos = [0.14, 0.0, 0.08]
    
    # Test with gripper specified
    result = get_inverse_kinematics(pos, gripper_value=75.0)
    assert result is not None, "Should find solution"
    assert result['gripper'] == 75.0, "Gripper should be 75"
    print("✓ Explicit gripper value propagated")
    
    # Test with default gripper
    result = get_inverse_kinematics(pos)
    assert result is not None, "Should find solution"
    assert result['gripper'] == 50.0, "Gripper should default to 50"
    print("✓ Default gripper value used")
    
    print("PASSED: Gripper value propagation")


def test_all_joints_within_limits():
    """Test that all computed joint angles respect limits."""
    print("\n=== Test: Joint Angles Within Limits ===")
    
    test_positions = [
        [0.15, 0.0, 0.09],
        [0.12, 0.08, 0.07],
        [0.10, -0.06, 0.10],
    ]
    
    for pos in test_positions:
        result = get_inverse_kinematics(pos)
        
        if result is None:
            continue  # Skip unreachable positions
        
        # Check all joints are within limits
        for joint_name, angle in result.items():
            min_val, max_val = JOINT_LIMITS[joint_name]
            assert min_val <= angle <= max_val, \
                f"{joint_name}={angle}° exceeds limits [{min_val}, {max_val}]"
        
        print(f"✓ All joints within limits for {pos}")
    
    print("PASSED: All joint angles within limits")


def run_all_tests():
    """Run all tests."""
    print("=" * 60)
    print("SO-101 Inverse Kinematics Solver - Unit Tests")
    print("=" * 60)
    
    tests = [
        test_workspace_validation,
        test_joint_limits,
        test_ik_known_positions,
        test_ik_returns_none_for_unreachable,
        test_ui_format_conversion,
        test_gripper_propagation,
        test_all_joints_within_limits,
    ]
    
    passed = 0
    failed = 0
    
    for test_fn in tests:
        try:
            test_fn()
            passed += 1
        except AssertionError as e:
            print(f"\n✗ FAILED: {test_fn.__name__}")
            print(f"  Error: {e}")
            failed += 1
        except Exception as e:
            print(f"\n✗ ERROR: {test_fn.__name__}")
            print(f"  Exception: {e}")
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
