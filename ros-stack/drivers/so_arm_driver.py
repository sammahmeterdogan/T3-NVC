#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import serial
import time

# Import IK solver from assignment8-so101 integration
try:
    from so101_inverse_kinematics import get_inverse_kinematics, convert_to_ui_format, check_workspace
except ImportError:
    get_inverse_kinematics = None
    convert_to_ui_format = None
    check_workspace = None

class SoArmDriver(Node):
    def __init__(self):
        super().__init__('so_arm_driver')
        
        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.ser = None
        self.init_serial_connection()

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            '/so_arm/command',
            self.listener_callback,
            10
        )
        self.get_logger().info('So Arm Driver initialized and waiting for commands...')

    def convert_ui_to_assignment_format(self, ui_joints):
        """Convert UI joint dict to assignment format."""
        return {
            'shoulder_pan': ui_joints.get('base', 0),
            'shoulder_lift': ui_joints.get('shoulder', 0),
            'elbow_flex': ui_joints.get('elbow', 0),
            'wrist_flex': ui_joints.get('wristPitch', 0),
            'wrist_roll': ui_joints.get('wristRoll', 0),
            'gripper': ui_joints.get('gripper', 50)
        }
    
    def convert_assignment_to_list(self, assignment_joints):
        """Convert assignment joint dict to ordered list for serial."""
        return [
            assignment_joints.get('shoulder_pan', 0),
            assignment_joints.get('shoulder_lift', 0),
            assignment_joints.get('elbow_flex', 0),
            assignment_joints.get('wrist_flex', 0),
            assignment_joints.get('wrist_roll', 0),
            assignment_joints.get('gripper', 50)
        ]

    def init_serial_connection(self):
        """Attempts to establish serial connection."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info("Connected to hardware")
        except serial.SerialException:
            self.ser = None
            self.get_logger().warn("Hardware not found - Running in Passive Mode")

    def listener_callback(self, msg):
        try:
            # 1. Parse incoming JSON
            command_frame = json.loads(msg.data)
            mode = command_frame.get('mode')
            data = command_frame.get('data')

            # 2. Handle JOINTS mode
            if mode == 'JOINTS':
                # Expected data: list [j1, j2, j3, j4, j5, j6] OR dict with UI joint names
                if isinstance(data, list):
                    # Direct list format
                    command_str = ",".join(map(str, data)) + "\n"
                    self.send_serial(command_str)
                elif isinstance(data, dict):
                    # Dict format from UI - convert to list
                    assignment_format = self.convert_ui_to_assignment_format(data)
                    joint_list = self.convert_assignment_to_list(assignment_format)
                    command_str = ",".join(map(str, joint_list)) + "\n"
                    self.send_serial(command_str)
                else:
                    self.get_logger().error(f"Invalid data format for JOINTS: {data}")

            # 3. Handle IK mode (Inverse Kinematics)
            elif mode == 'IK':
                if get_inverse_kinematics is None:
                    self.get_logger().error("IK Solver module not found - check so101_inverse_kinematics.py")
                    return
                
                # Expected data: dict with x, y, z (in meters), optional pitch/roll/yaw
                if not isinstance(data, dict):
                    self.get_logger().error(f"Invalid data format for IK: {data}")
                    return
                
                # Extract position
                x = data.get('x', 0.0)
                y = data.get('y', 0.0)
                z = data.get('z', 0.0)
                target_position = [x, y, z]
                
                # Optional: extract orientation (not implemented in simplified IK)
                # pitch = data.get('pitch', 0)
                # roll = data.get('roll', 0)
                # yaw = data.get('yaw', 0)
                
                # Optional: gripper value
                gripper = data.get('gripper', None)
                
                self.get_logger().info(f"[IK] Computing solution for position: {target_position}")
                
                # Check workspace first
                if check_workspace and not check_workspace(target_position):
                    self.get_logger().warn(f"[IK] Position {target_position} is outside workspace")
                    return
                
                # Compute IK solution
                joint_config = get_inverse_kinematics(target_position, gripper_value=gripper)
                
                if joint_config is None:
                    self.get_logger().error(f"[IK] No solution found for position {target_position}")
                    return
                
                # Log solution
                self.get_logger().info(f"[IK] Solution found:")
                for joint_name, angle in joint_config.items():
                    self.get_logger().info(f"  {joint_name:15s}: {angle:7.2f}°")
                
                # Convert to list and send
                joint_list = self.convert_assignment_to_list(joint_config)
                command_str = ",".join(map(str, joint_list)) + "\n"
                self.send_serial(command_str)
                
            else:
                self.get_logger().warn(f"Unknown mode received: {mode}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

    def send_serial(self, command_string):
        """Writes command to serial port if connected."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command_string.encode('utf-8'))
                self.get_logger().info(f"[INFO] Sent to Serial: {command_string.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication failed: {e}")
                self.ser.close()
                self.ser = None
                self.get_logger().warn("Switched to Passive Mode (Connection lost)")
        else:
            # In Passive Mode, we just acknowledge the command logically
            self.get_logger().info(f"[PASSIVE] Command processed: {command_string.strip()}")

def main(args=None):
    rclpy.init(args=args)
    driver = SoArmDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if driver.ser and driver.ser.is_open:
            driver.ser.close()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
