import time
import math
from .PCA9685 import PCA9685
from .ADC import *

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.adc = Adc()

        # -----------------------------
        # Motor inversion flags
        # Set True if that motor spins backward
        # when given a positive command
        # -----------------------------
        self.invert_FL = False
        self.invert_BL = False
        self.invert_FR = False  
        self.invert_BR = True

        self.MAX_PWM = 4095

    # -----------------------------
    # Internal helpers
    # -----------------------------
    def _apply_inversion(self, value, invert):
        return -value if invert else value

    def _normalize(self, FL, BL, FR, BR):
        max_val = max(abs(FL), abs(BL), abs(FR), abs(BR), 1)
        scale = min(self.MAX_PWM / max_val, 1.0)

        return (
            int(FL * scale),
            int(BL * scale),
            int(FR * scale),
            int(BR * scale),
        )

    def _set_wheel_speeds(self, FL, BL, FR, BR):
        """Set individual wheel speeds with inversion + safety."""

        # Apply inversion
        FL = self._apply_inversion(FL, self.invert_FL)
        BL = self._apply_inversion(BL, self.invert_BL)
        FR = self._apply_inversion(FR, self.invert_FR)
        BR = self._apply_inversion(BR, self.invert_BR)

        # Normalize to avoid clipping
        FL, BL, FR, BR = self._normalize(FL, BL, FR, BR)

        # Send to motor driver (H-bridge style)
        self.pwm.setMotorPwm(0, max(0, FL))
        self.pwm.setMotorPwm(1, max(0, -FL))

        self.pwm.setMotorPwm(2, max(0, BL))
        self.pwm.setMotorPwm(3, max(0, -BL))

        self.pwm.setMotorPwm(4, max(0, FR))
        self.pwm.setMotorPwm(5, max(0, -FR))

        self.pwm.setMotorPwm(6, max(0, BR))
        self.pwm.setMotorPwm(7, max(0, -BR))

    # -----------------------------
    # Public API
    # -----------------------------
    def move(self, angle, speed=50):
        """
        Move the robot.

        angle: 0° = forward, 180° = backward
        speed: 0–100 (%)
        """
        speed_pwm = int(speed * 40.95)
        rad = math.radians(angle)

        VY = speed_pwm * math.cos(rad)
        VX = speed_pwm * math.sin(rad)

        # Differential-style mixing
        FL = VY + VX
        BL = VY - VX
        FR = VY - VX
        BR = VY + VX

        self._set_wheel_speeds(FL, BL, FR, BR)

    def rotate(self, direction, speed=50):
        """
        Rotate robot in place.

        direction: "left" or "right"
        speed: 0–100 (%)
        """
        speed_pwm = int(speed * 40.95)

        if direction == "left":
            self._set_wheel_speeds(-speed_pwm, -speed_pwm,
                                   speed_pwm,  speed_pwm)
        elif direction == "right":
            self._set_wheel_speeds(speed_pwm,  speed_pwm,
                                   -speed_pwm, -speed_pwm)

    def stop(self):
        """Stop all motors."""
        self._set_wheel_speeds(0, 0, 0, 0)
