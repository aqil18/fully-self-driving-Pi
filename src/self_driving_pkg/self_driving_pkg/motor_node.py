#!/usr/bin/env python3

## Subscribes to the motor topic and acts on the commands.

## Aim:
# 1. Receives angle commands from the motor topic, e.g., /motor/cmd
# 2. Acts on the commands, e.g., Right 23 degrees, Left 43 degrees

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from .car_utils.Motor_api import Motor  # Ensure this path is correct
import json

SPEED = 25  # Default speed percentage
class MotorActionNode(Node):
    def __init__(self):
        super().__init__('motor_action_node')
        self.subscription = self.create_subscription(
            Int32, '/motor/cmd', self.motor_callback, 10)
        self.get_logger().info("Motor action node has started!")
        self.motor = Motor()

        
    def motor_callback(self, msg):
        """Handle incoming ROS 2 messages."""
        try:
            angle = msg.data
            self.get_logger().info(f"Received motor angle: {angle}")            
            self.motor.move(angle, speed=SPEED)
            self.get_logger().info(f'Moving at {angle}° with {SPEED}% speed')

        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.motor.stop()  # Stop the motor on shutdown
        node.get_logger().info("Shutting down motor action node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
