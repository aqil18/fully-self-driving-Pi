import time
import math
from .PCA9685 import PCA9685
from .ADC import *

## API example for the motor control

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.adc = Adc()
        self.fr_inverted = False  # Set to True if front-right motor is inverted
        self.fl_inverted = False  # Set to True if front-left motor is inverted
        self.br_inverted = True  # Set to True if back-right motor is inverted
        self.bl_inverted = False  # Set to True if back-left motor is inverted
        
    
    def _set_wheel_speeds(self, FL, BL, FR, BR):
        """Set individual wheel speeds."""
        if self.fl_inverted:
            FL = -FL
        if self.fr_inverted:
            FR = -FR
        if self.bl_inverted:
            BL = -BL
        if self.br_inverted:
            BR = -BR
        
        self.pwm.setMotorPwm(0, max(0, FL))
        self.pwm.setMotorPwm(1, max(0, -FL))
        self.pwm.setMotorPwm(2, max(0, BL))
        self.pwm.setMotorPwm(3, max(0, -BL))
        self.pwm.setMotorPwm(4, max(0, BR))
        self.pwm.setMotorPwm(5, max(0, -BR))
        self.pwm.setMotorPwm(6, max(0, FR))
        self.pwm.setMotorPwm(7, max(0, -FR))


    def move(self, angle, speed=50):
        """
        Move the robot in a specified direction.

        :param angle: Movement direction (0° = forward, 180° = backward)
        :param speed: Speed percentage (0-100)
        """
        speed = int(speed * 40.95)  # Convert to motor duty range (0-4095)
        # Python trig functions use radians, so this is required.
        rad_angle = math.radians(angle)
        
        # Calculate wheel speeds based on direction
        # Speed is a magnitude here
        VY = int(speed * math.cos(rad_angle)) # forward/backward component
        VX = int(speed * math.sin(rad_angle)) # left/right component (used here as turning)
        
        # Differential drive formula to make the robot move in the desired direction
        # left_side  = forward + turn
        # right_side = forward - turn
        FL = VY + VX
        BL = VY + VX
        FR = VY - VX
        BR = VY - VX
        
        self._set_wheel_speeds(FL, BL, FR, BR)
        
        
    def rotate(self, direction, speed=50):
        """
        Rotate the robot.

        :param direction: "left" or "right"
        :param speed: Speed percentage (0-100)
        """
        speed = int(speed * 40.95)  # Convert to motor duty range (0-4095)
        
        if direction == "left":
            self._set_wheel_speeds(-speed, -speed, speed, speed)
        elif direction == "right":
            self._set_wheel_speeds(speed, speed, -speed, -speed)

    def stop(self):
        """Stop the robot."""
        self._set_wheel_speeds(0, 0, 0, 0)


def main():
    motor = Motor()

    print("\nMotor Control Interface")
    print("------------------------")
    print("Commands:")
    print("  move <angle> <speed>   → angle: 0–360, speed: 0–100")
    print("  rotate <left|right> <speed>")
    print("  stop")
    print("  quit\n")

    try:
        while True:
            cmd = input(">> ").strip().lower()
            if not cmd:
                continue

            parts = cmd.split()

            if parts[0] == "move" and len(parts) == 3:
                angle = float(parts[1])
                speed = float(parts[2])
                motor.move(angle, speed)

            elif parts[0] == "rotate" and len(parts) == 3:
                direction = parts[1]
                speed = float(parts[2])
                motor.rotate(direction, speed)

            elif parts[0] == "stop":
                motor.stop()

            elif parts[0] in ("quit", "exit"):
                motor.stop()
                break

            else:
                print("Invalid command.")

    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        print("\nMotors stopped. Exiting.")

def main2():
    motor = Motor()

    import sys
    import termios
    import tty
    import select
    import time

    # Keyboard setup
    settings = termios.tcgetattr(sys.stdin)

    max_speed = 40 
    speed = 0
    step_speed = 10        # base speed %
    max_angle = 30      # degrees
    angle = 0
    angle_step = 20

    def get_key():
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    print("\nWASD Motor Teleop")
    print("----------------")
    print("W/S : forward / backward")
    print("A/D : steer left / right")
    print("SPACE : stop")
    print("CTRL+C : quit\n")

    try:
        while True:
            key = get_key()

            if key == 'w':
                speed = min(speed + step_speed, max_speed)
                angle = 0
            elif key == 's':
                speed = max(speed - step_speed, max_speed)
                angle = 180
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
            
            if speed < 0:
                speed = -speed

            # Combine forward/backward + steering
            motor.move(angle, speed)

            time.sleep(0.05)  # ~20 Hz loop

    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        print("\nMotors stopped. Exiting.")


if __name__ == "__main__":
    main2()

