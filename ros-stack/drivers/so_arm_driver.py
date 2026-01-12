#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import serial
import time

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
                # Expected data: list of integers/floats [j1, j2, j3, j4, j5, j6]
                if isinstance(data, list):
                    # Convert list to CSV string terminated by newline
                    # e.g., "90,45,120,0,0,0\n"
                    command_str = ",".join(map(str, data)) + "\n"
                    self.send_serial(command_str)
                else:
                    self.get_logger().error(f"Invalid data format for JOINTS: {data}")

            # 3. Handle IK mode (Inverse Kinematics)
            elif mode == 'IK':
                self.get_logger().warn("IK Solver not implemented on Driver yet")
                
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
