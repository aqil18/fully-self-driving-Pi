#!/usr/bin/env python3

## Subscribes to the motor topic and acts on the commands.

## Aim:
# 1. Receives angle commands from the motor topic, e.g., /motor/cmd
# 2. Acts on the commands, e.g., Right 23 degrees, Left 43 degrees

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorActionNode(Node):
    def __init__(self):
        super().__init__('motor_action_node')
        self.subscription = self.create_subscription(
            String, '/motor/cmd', self.motor_callback, 10)
        self.get_logger().info("Motor action node has started!")

    def motor_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received motor command: {command}")
        if command == "stop":
            self.get_logger().info("Stopping motor")
        elif command == "forward":
            self.get_logger().info("Moving forward")
        elif command.startswith("Right"):
            angle = int(command.split()[1])
            self.get_logger().info(f"Turning right {angle} degrees")
        elif command.startswith("Left"):
            angle = int(command.split()[1])
            self.get_logger().info(f"Turning left {angle} degrees")

def main(args=None):
    rclpy.init(args=args)
    node = MotorActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
