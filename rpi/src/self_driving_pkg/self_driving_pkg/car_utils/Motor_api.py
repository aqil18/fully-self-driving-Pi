from PCA9685 import PCA9685
from ADC import *

MAX_SPEED  = 40    # matches teleop max_speed
MIN_SPEED  = -40
MAX_PWM    = 4095
STEER_GAIN       = 3  # increase for sharper turns (1.0 = linear, 2.0 = very aggressive)
DEADBAND_FRONT   = 0    # front motors start without a boost
DEADBAND_REAR    = 800  # tune between 600-800 until rear motors reliably start
SMOOTH           = 0  # 0.0 = no smoothing (instant), 1.0 = never moves — tune between 0.3-0.7
STEP_ANGLE = 30

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.adc = Adc()
        self.fr_inverted = False
        self.fl_inverted = False
        self.br_inverted = True
        self.bl_inverted = False
        self._left_pwm  = 0.0
        self._right_pwm = 0.0

    def _apply_deadband(self, v, deadband):
        if v > 0: return max(v, deadband)
        if v < 0: return min(v, -deadband)
        return 0

    def _set_wheel_speeds(self, FL, BL, FR, BR):
        if self.fl_inverted: FL = -FL
        if self.fr_inverted: FR = -FR
        if self.bl_inverted: BL = -BL
        if self.br_inverted: BR = -BR

        FL = self._apply_deadband(FL, DEADBAND_FRONT)
        FR = self._apply_deadband(FR, DEADBAND_FRONT)
        BL = self._apply_deadband(BL, DEADBAND_REAR)
        BR = self._apply_deadband(BR, DEADBAND_REAR)

        self.pwm.setMotorPwm(0, max(0, min(MAX_PWM,  FL)))
        self.pwm.setMotorPwm(1, max(0, min(MAX_PWM, -FL)))
        self.pwm.setMotorPwm(2, max(0, min(MAX_PWM,  BL)))
        self.pwm.setMotorPwm(3, max(0, min(MAX_PWM, -BL)))
        self.pwm.setMotorPwm(4, max(0, min(MAX_PWM,  BR)))
        self.pwm.setMotorPwm(5, max(0, min(MAX_PWM, -BR)))
        self.pwm.setMotorPwm(6, max(0, min(MAX_PWM,  FR)))
        self.pwm.setMotorPwm(7, max(0, min(MAX_PWM, -FR)))

    def move(self, angle, speed):
        """
        Move the robot.

        :param angle: Steering angle (-90 = full left, 0 = straight, 90 = full right)
        :param speed: Speed (0-40 forward, negative = straight backward)
        """
        pwm = int(abs(speed) * (MAX_PWM / MAX_SPEED))

        if speed < 0:
            self._set_wheel_speeds(-pwm, -pwm, -pwm, -pwm)
            return

        turn = (angle / 90.0) * STEER_GAIN

        target_left  = pwm * (1 + turn)
        target_right = pwm * (1 - turn)

        self._left_pwm  = self._left_pwm  * SMOOTH + target_left  * (1 - SMOOTH)
        self._right_pwm = self._right_pwm * SMOOTH + target_right * (1 - SMOOTH)

        self._set_wheel_speeds(int(self._left_pwm), int(self._left_pwm),
                               int(self._right_pwm), int(self._right_pwm))

    def stop(self):
        self._left_pwm  = 0.0
        self._right_pwm = 0.0
        self._set_wheel_speeds(0, 0, 0, 0)


if __name__ == "__main__":
    import sys, termios, tty

    motor = Motor()

    speed = 0
    angle = 0
    step_speed = 2
    max_speed  = MAX_SPEED
    min_speed = MIN_SPEED
    step_angle = STEP_ANGLE
    max_angle  = 180

    def getch():
        fd = sys.stdin.fileno()
        orig = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, orig)

    print("\nWASD Motor Teleop")
    print("W/S : speed up / slow down")
    print("A/D : steer left / right")
    print("SPACE : stop  |  CTRL+C : quit\n")

    try:
        while True:
            key = getch()
            if key == 'w':
                speed = min(speed + step_speed, max_speed)
            elif key == 's':
                speed = max(speed - step_speed, min_speed )
            elif key == 'a':
                angle = max(angle - step_angle, -max_angle)
            elif key == 'd':
                angle = min(angle + step_angle,  max_angle)
            elif key == ' ':
                speed, angle = 0, 0
                motor.stop()
                continue
            elif key == '\x03':
                break

            print(f"Speed: {speed}  Angle: {angle}")
            motor.move(angle, speed)

    except KeyboardInterrupt:
        pass
    finally:
        motor.stop()
        print("Motors stopped.")