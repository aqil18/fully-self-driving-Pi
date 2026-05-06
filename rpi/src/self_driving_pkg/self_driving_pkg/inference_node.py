#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from interfaces.msg import Motor
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import torch

from .ml.pipilotnet import PiPilotNet
from .ml.preprocessor import PreProcessor
from .config import Config

# Must match CAMERA_W in ml/train.py
CAMERA_W = 640


class LaneInferenceNode(Node):
    def __init__(self):
        super().__init__('lane_inference_node')

        self.motor_pub = self.create_publisher(Motor, '/motor/cmd', 10)
        self.image_pub = self.create_publisher(Image, '/inference/image_out', 10)

        self.create_subscription(Image, '/fsd/image_raw', self.image_callback, 10)
        self.create_subscription(Int32, '/detection/offset', self.offset_callback, 10)

        self.bridge = CvBridge()
        self.cfg = Config()

        # Cache the latest offset from the detection node (pixels, unnormalised)
        self._latest_offset_px: int = 0

        # Temporal smoothing
        self.publish_every_n_frames = 5
        self.frame_count = 0
        self.steering_buffer = deque(maxlen=self.publish_every_n_frames)
        self.throttle_buffer = deque(maxlen=self.publish_every_n_frames)

        self.model = PiPilotNet()
        self.preprocessor = PreProcessor()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        ckpt = torch.load(self.cfg.model_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(ckpt["model_state"])
        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info("Lane inference node started.")

    def offset_callback(self, msg: Int32):
        self._latest_offset_px = msg.data

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Preprocess image
        rgb = self.preprocessor.preprocess(frame)
        x = torch.from_numpy(np.transpose(rgb, (2, 0, 1))).unsqueeze(0).float().to(self.device)

        # Normalise offset to [-1, 1] — same scheme as training
        offset_norm = float(np.clip(self._latest_offset_px / (CAMERA_W / 2), -1.0, 1.0))
        offset_t = torch.tensor([[offset_norm]], dtype=torch.float32).to(self.device)

        with torch.no_grad():
            pred_steering, pred_throttle = self.model(x, offset_t)

        steering_deg = float(pred_steering.item() * self.cfg.max_angle)
        throttle_val = float(pred_throttle.item() * self.cfg.max_throttle)

        self.steering_buffer.append(steering_deg)
        self.throttle_buffer.append(throttle_val)
        self.frame_count += 1

        if self.frame_count % self.publish_every_n_frames == 0:
            smoothed_steering = float(np.mean(self.steering_buffer))
            smoothed_throttle = float(np.mean(self.throttle_buffer))

            motor_msg = Motor()
            motor_msg.angle = int(smoothed_steering)
            motor_msg.speed = int(smoothed_throttle)
            self.motor_pub.publish(motor_msg)

            self.get_logger().info(
                f"offset={self._latest_offset_px:+d}px  "
                f"steering={smoothed_steering:.1f}  throttle={smoothed_throttle:.1f}"
            )

            # Annotated image for debugging
            out = frame.copy()
            cv2.putText(out, f"Steer: {smoothed_steering:.1f} deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(out, f"Throttle: {smoothed_throttle:.1f}",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(out, f"Offset: {self._latest_offset_px:+d}px",
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))


def main():
    rclpy.init()
    node = LaneInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motor_pub.publish(Motor())  # zero command on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
