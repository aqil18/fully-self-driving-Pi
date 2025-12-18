#!/usr/bin/env python3

## Subscribes to the motor topic and acts on the commands.

## Aim:
# 1. Receives angle commands from the motor topic, e.g., /motor/cmd
# 2. Acts on the commands, e.g., Right 23 degrees, Left 43 degrees

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .car_utils.Motor_api import Motor  # Ensure this path is correct
import json


class MotorActionNode(Node):
    def __init__(self):
        super().__init__('motor_action_node')
        self.subscription = self.create_subscription(
            String, '/motor/cmd', self.motor_callback, 10)
        self.get_logger().info("Motor action node has started!")
        self.motor = Motor()

        
    def motor_callback(self, msg):
        """Handle incoming ROS 2 messages."""
        try:
            command = json.loads(str(msg))

            self.get_logger().info(f"Received motor command: {command}")            
            action = command.get("action", "")
            angle = command.get("angle", 0)
            speed = command.get("speed", 50)

            if action == "move":
                self.motor.move(angle, speed)
                self.get_logger().info(f'Moving at {angle}° with {speed}% speed')
            elif action == "rotate":
                self.motor.rotate("left" if angle < 0 else "right", speed)
                self.get_logger().info(f'Rotating {"left" if angle < 0 else "right"} at {speed}% speed')
            elif action == "stop":
                self.motor.stop()
                self.get_logger().info('Stopping motors')
            else:
                self.get_logger().warn('Invalid action received')

        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
