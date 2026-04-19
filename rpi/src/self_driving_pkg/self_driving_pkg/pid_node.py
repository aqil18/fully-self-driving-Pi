#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from interfaces.msg import Motor
import time

# ── Tune these ────────────────────────────────────────────────────────────────
DRIVE_SPEED   = 6      # fixed forward speed (0–40)
Kp            = 0.08    # proportional gain  (start small, increase until it steers)
Ki            = 0.0     # integral gain      (add if it drifts consistently to one side)
Kd            = 0.02    # derivative gain    (add to dampen oscillation)
MAX_ANGLE     = 90      # clamp output to ±90°
WATCHDOG_SECS = 0.5     # stop if no offset received within this window
# ─────────────────────────────────────────────────────────────────────────────


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        self.motor_pub = self.create_publisher(Motor, '/motor/cmd', 10)
        self.create_subscription(Int32, '/detection/offset', self.offset_callback, 10)

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()
        self._last_msg_time = time.monotonic()

        # Watchdog: stop motors if detection goes silent
        self.create_timer(WATCHDOG_SECS, self._watchdog)

        self.get_logger().info("PID node started  (Kp={} Ki={} Kd={})".format(Kp, Ki, Kd))

    # ------------------------------------------------------------------
    def offset_callback(self, msg):
        now   = time.monotonic()
        error = float(msg.data)          # pixels; positive = centroid is right of centre
        dt    = now - self._prev_time
        if dt <= 0:
            dt = 1e-3

        self._integral   += error * dt
        derivative        = (error - self._prev_error) / dt
        raw_angle         = Kp * error + Ki * self._integral + Kd * derivative

        # Clamp and negate: positive offset → steer left (negative angle)
        angle = int(max(-MAX_ANGLE, min(MAX_ANGLE, -raw_angle)))

        self._prev_error    = error
        self._prev_time     = now
        self._last_msg_time = now

        motor_msg         = Motor()
        motor_msg.speed   = DRIVE_SPEED
        motor_msg.angle   = angle
        self.motor_pub.publish(motor_msg)

        self.get_logger().info(f"offset={msg.data:+d}  angle={angle:+d}  integral={self._integral:.1f}")

    # ------------------------------------------------------------------
    def _watchdog(self):
        if time.monotonic() - self._last_msg_time > WATCHDOG_SECS:
            stop = Motor()
            stop.speed = 0
            stop.angle = 0
            self.motor_pub.publish(stop)
            self._integral   = 0.0
            self._prev_error = 0.0


def main():
    rclpy.init()
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Motor()
        stop.speed = 0
        stop.angle = 0
        node.motor_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()