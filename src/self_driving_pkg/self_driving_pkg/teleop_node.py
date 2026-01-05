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


    def getch():
        import sys, termios, tty

        fd = sys.stdin.fileno()
        orig = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)  # or tty.setraw(fd) if you prefer raw mode's behavior.
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, orig)



    def main2():
        speed = 0
        step_speed = 10     # base speed %
        max_speed = 40 
        angle = 0
        angle_step = 20
        max_angle = 90      # degrees

        
        print("\nWASD Motor Teleop")
        print("----------------")
        print("W/S : forward / backward")
        print("A/D : steer left / right")
        print("SPACE : stop")
        print("CTRL+C : quit\n")
        
        try:
            while True:
                key = getch()

                if key == 'w':
                    speed = min(speed + step_speed, max_speed)
                elif key == 's':
                    speed = max(speed - step_speed, -max_speed)
                elif key == 'a':
                    angle = max(angle - angle_step, -max_angle)
                elif key == 'd':
                    angle = min(angle + angle_step, max_angle)
                elif key == ' ':
                    motor.stop()
                    speed = 0
                    angle = 0
                    continue
                elif key == '\x03':  # Ctrl+C
                    break

        
                cmdAngle = angle
                cmdSpeed = abs(speed)
                if speed < 0:
                    cmdAngle = -cmdAngle + 180


                print(f"Speed: {cmdSpeed}%, Angle: {cmdAngle}°")
                # Combine forward/backward + steering
                # !! publish speed and angle motor.move(cmdAngle, cmdSpeed)

            
            self.get_logger().info(
                f'Steering: {self.steering} degrees'
            )
            self.steer_pub.publish(Int32(data=self.steering))


        except KeyboardInterrupt:
            pass
        finally:
            motor.stop()
            print("\nMotors stopped. Exiting.")

def main():
    rclpy.init()
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
