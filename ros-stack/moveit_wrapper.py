#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MoveItWrapper(Node):
    def __init__(self):
        super().__init__('moveit_wrapper')
        self.subscription = self.create_subscription(
            String,
            '/so_arm/moveit_cmd',
            self.listener_callback,
            10)
        self.get_logger().info('MoveIt Wrapper Node Started. Listening on /so_arm/moveit_cmd')

    def listener_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f'Received command: {cmd}')
        
        # In a real scenario, this would use moveit_commander or MoveGroupInterface
        # Since we are in a customized environment, we will Simulate the behavior
        # and attempt to call standard ROS 2 CLI tools for actions if possible.
        
        if cmd == 'plan':
            self.get_logger().info('EXECUTING: Planning path (Simulated)...')
            # Add logic here to trigger 'Plan' action
        elif cmd == 'execute':
            self.get_logger().info('EXECUTING: Executing path (Simulated)...')
        elif cmd == 'stop':
            self.get_logger().info('EXECUTING: STOP command...')
        elif cmd == 'home':
            self.get_logger().info('EXECUTING: Moving to HOME...')
        
        # Feedback for UI
        # self.get_logger().info('Action Complete')

def main(args=None):
    rclpy.init(args=args)
    wrapper = MoveItWrapper()
    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
