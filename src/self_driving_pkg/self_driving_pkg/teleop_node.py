#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import termios
import tty
import select

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.steer_pub = self.create_publisher(Int32, '/motor/cmd', 10)

        self.steering = 0

        self.max_steer = 30
        self.step = 2

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "Keyboard teleop:\n"
            "  A/D : steer left/right\n"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()

        if key == 'a':
            self.steering = max(self.steering - self.step, -self.max_steer)
        elif key == 'd':
            self.steering = min(self.steering + self.step, self.max_steer)
        elif key == ' ':
            self.steering = 0

        self.steer_pub.publish(Int32(data=self.steering))

def main():
    rclpy.init()
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
