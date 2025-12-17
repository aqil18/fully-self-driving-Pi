import time
import math
from PCA9685 import PCA9685
from ADC import *

## API example for the motor control

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.adc = Adc()
    
    def _set_wheel_speeds(self, FL, BL, FR, BR):
        """Set individual wheel speeds."""
        self.pwm.setMotorPwm(0, max(0, FL))
        self.pwm.setMotorPwm(1, max(0, -FL))
        self.pwm.setMotorPwm(2, max(0, BL))
        self.pwm.setMotorPwm(3, max(0, -BL))
        self.pwm.setMotorPwm(4, max(0, FR))
        self.pwm.setMotorPwm(5, max(0, -FR))
        self.pwm.setMotorPwm(6, max(0, BR))
        self.pwm.setMotorPwm(7, max(0, -BR))

    def move(self, angle, speed=50):
        """
        Move the robot in a specified direction.

        :param angle: Movement direction (0° = forward, 180° = backward)
        :param speed: Speed percentage (0-100)
        """
        speed = int(speed * 40.95)  # Convert to motor duty range (0-4095)
        rad_angle = math.radians(angle)
        
        # Calculate wheel speeds based on direction
        VY = int(speed * math.cos(rad_angle))
        VX = int(speed * math.sin(rad_angle))
        
        # Differential drive formula
        FL = VY + VX
        BL = VY - VX
        FR = VY - VX
        BR = VY + VX
        
        self._set_wheel_speeds(FL, BL, FR, BR)
        print(f"MOVING") # Need to use the ros2 logger here for this to work 
        
        
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


# Example Usage
if __name__ == "__main__":
    motor = Motor()

    try:
        motor.move(0, 50)     # Move forward at 50% speed
        time.sleep(2)
        motor.move(180, 50)   # Move backward at 50% speed
        time.sleep(2)
        motor.rotate("left", 40)  # Rotate left
        time.sleep(2)
        motor.stop()          # Stop the robot
    except KeyboardInterrupt:
        motor.stop()
