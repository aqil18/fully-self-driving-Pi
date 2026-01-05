#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Motor
import sys
import termios
import tty
import select

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.steer_pub = self.create_publisher(Motor, '/motor/cmd', 10)


        self.speed = 0
        self.step_speed = 10     # base speed %
        self.max_speed = 40
        self.angle = 0
        self.angle_step = 15
        self.max_angle = 90      # degrees

        # Do we need a timer here?
        # self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.loop()

    def getch(self):
        import sys, termios, tty

        fd = sys.stdin.fileno()
        orig = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)  # or tty.setraw(fd) if you prefer raw mode's behavior.
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, orig)



    def loop(self):
        print("\nWASD Motor Teleop")
        print("----------------")
        print("W/S : forward / backward")
        print("A/D : steer left / right")
        print("SPACE : stop")
        print("CTRL+C : quit\n")
        
        while True:
            key = self.getch()

            if key == 'w':
                self.speed = min(self.speed + self.step_speed, self.max_speed)
            elif key == 's':
                self.speed = max(self.speed - self.step_speed, -self.max_speed)
            elif key == 'a':
                self.angle = max(self.angle - self.angle_step, -self.max_angle)
            elif key == 'd':
                self.angle = min(self.angle + self.angle_step, self.max_angle)
            elif key == ' ':
                self.speed = 0
                self.angle = 0
                continue
            elif key == '\x03':  # Ctrl+C
                break


            cmdAngle = self.angle
            cmdSpeed = abs(self.speed)
            if self.speed < 0:
                cmdAngle = -cmdAngle + 180

            self.get_logger().info(
                f"Speed: {cmdSpeed}%, Angle: {cmdAngle}°"
            )
            msg = Motor()
            msg.speed = cmdSpeed
            msg.angle = cmdAngle
            self.steer_pub.publish(msg)



def main():
    rclpy.init()
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
