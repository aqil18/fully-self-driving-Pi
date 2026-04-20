#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from interfaces.msg import Motor
import time

# ── Tune these ────────────────────────────────────────────────────────────────
DRIVE_SPEED       = 4      # fixed forward speed (0–40)
Kp                = 0.35   # proportional gain
Ki                = 0.0    # integral gain
Kd                = 0.02   # derivative gain
MAX_ANGLE         = 90     # clamp output to ±90°
WATCHDOG_SECS     = 0.5    # switch to CNN if no offset received within this window
HYSTERESIS_COUNT  = 3      # consecutive valid offsets required to switch back to perception
# ─────────────────────────────────────────────────────────────────────────────


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        self.motor_pub = self.create_publisher(Motor, '/motor/cmd', 10)
        self.create_subscription(Int32, '/detection/offset',  self.offset_callback,  10)
        self.create_subscription(Int32, '/inference/steering', self.cnn_callback, 10)

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()

        self._last_offset_time  = time.monotonic()
        self._cnn_angle         = 0
        self._mode              = 'perception'   # 'perception' | 'cnn'
        self._consecutive_valid = 0              # hysteresis counter

        self.create_timer(WATCHDOG_SECS, self._watchdog)

        self.get_logger().info(
            f"PID node started  Kp={Kp} Ki={Ki} Kd={Kd}  fallback=CNN"
        )

    # ------------------------------------------------------------------
    def cnn_callback(self, msg):
        self._cnn_angle = msg.data

    # ------------------------------------------------------------------
    def offset_callback(self, msg):
        now   = time.monotonic()
        error = float(msg.data)
        dt    = now - self._prev_time
        if dt <= 0:
            dt = 1e-3

        # Hysteresis: count consecutive valid offsets before trusting perception again
        self._consecutive_valid += 1
        if self._mode == 'cnn' and self._consecutive_valid >= HYSTERESIS_COUNT:
            self._mode = 'perception'
            self._integral   = 0.0
            self._prev_error = 0.0
            self.get_logger().info("Switched to PERCEPTION mode.")

        self._last_offset_time = now

        if self._mode != 'perception':
            return

        self._integral += error * dt
        derivative      = (error - self._prev_error) / dt
        raw_angle       = Kp * error + Ki * self._integral + Kd * derivative

        angle = int(max(-MAX_ANGLE, min(MAX_ANGLE, raw_angle)))

        self._prev_error = error
        self._prev_time  = now

        self._publish(angle, DRIVE_SPEED)
        self.get_logger().info(
            f"[PID] angle={angle:+d} speed={DRIVE_SPEED}"
        )

    # ------------------------------------------------------------------
    def _watchdog(self):
        if time.monotonic() - self._last_offset_time > WATCHDOG_SECS:
            self._consecutive_valid = 0
            if self._mode == 'perception':
                self._mode = 'cnn'
                self._integral   = 0.0
                self._prev_error = 0.0
                self.get_logger().warn("Line lost — switched to CNN fallback.")

            if self._mode == 'cnn':
                self._publish(self._cnn_angle, DRIVE_SPEED)
                self.get_logger().info(
                    f"[CNN] angle={self._cnn_angle:+d}"
                )

    # ------------------------------------------------------------------
    def _publish(self, angle, speed):
        msg       = Motor()
        msg.angle = angle
        msg.speed = speed
        self.motor_pub.publish(msg)


def main():
    rclpy.init()
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
