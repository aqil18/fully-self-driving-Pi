#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import termios
import tty
import select

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.steer_pub = self.create_publisher(Int32, '/vehicle/steering', 10)
        self.throttle_pub = self.create_publisher(Int32, '/vehicle/throttle', 10)

        self.steering = 0
        self.throttle = 0

        self.max_steer = 30
        self.max_throttle = 100
        self.step = 5

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "Keyboard teleop:\n"
            "  W/S : throttle up/down\n"
            "  A/D : steer left/right\n"
            "  SPACE: stop\n"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()

        if key == 'w':
            self.throttle = min(self.throttle + self.step, self.max_throttle)
        elif key == 's':
            self.throttle = max(self.throttle - self.step, -self.max_throttle)
        elif key == 'a':
            self.steering = max(self.steering - self.step, -self.max_steer)
        elif key == 'd':
            self.steering = min(self.steering + self.step, self.max_steer)
        elif key == ' ':
            self.throttle = 0
            self.steering = 0

        self.steer_pub.publish(Int32(data=self.steering))
        self.throttle_pub.publish(Int32(data=self.throttle))

def main():
    rclpy.init()
    node = TeleopKeyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import termios
import tty
import select

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.steer_pub = self.create_publisher(Int32, '/vehicle/steering', 10)
        self.throttle_pub = self.create_publisher(Int32, '/vehicle/throttle', 10)

        self.steering = 0
        self.throttle = 0

        self.max_steer = 30
        self.max_throttle = 100
        self.step = 5

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "Keyboard teleop:\n"
            "  W/S : throttle up/down\n"
            "  A/D : steer left/right\n"
            "  SPACE: stop\n"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()

        if key == 'w':
            self.throttle = min(self.throttle + self.step, self.max_throttle)
        elif key == 's':
            self.throttle = max(self.throttle - self.step, -self.max_throttle)
        elif key == 'a':
            self.steering = max(self.steering - self.step, -self.max_steer)
        elif key == 'd':
            self.steering = min(self.steering + self.step, self.max_steer)
        elif key == ' ':
            self.throttle = 0
            self.steering = 0

        self.steer_pub.publish(Int32(data=self.steering))
        self.throttle_pub.publish(Int32(data=self.throttle))

def main():
    rclpy.init()
    node = TeleopKeyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
