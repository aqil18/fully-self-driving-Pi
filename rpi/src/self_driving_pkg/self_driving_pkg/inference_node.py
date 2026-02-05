#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Motor
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from collections import deque
import torch
from models import train


PATH = '/models/model.pt' # The path where your model weights are saved



class LaneInferenceNode(Node):
    def __init__(self):
        super().__init__('lane_inference_node')

        self.motor_pub = self.create_publisher(Motor, '/motor/cmd', 10)
        self.image_pub = self.create_publisher(Image, '/inference/image_out', 10)
        self.create_subscription(Image, '/fsd/image_raw', self.inference_callback, 10)

        self.bridge = CvBridge()
        
        # Temporal smoothing
        self.frame_count = 0
        self.publish_every_n_frames = 7  # choose 5–10
        self.steering_buffer = deque(maxlen=self.publish_every_n_frames)
        self.smoothed_angle = 0.0

        # Convolutional Neural Network
        model = train.PiPilotNet()
        ckpt = torch.load(PATH, map_location=self.device)
        # Use weights_only=True for best practice when loading weights
        model.load_state_dict(ckpt["model_state"])
        model.eval()

        
        self.get_logger().info("Lane inference node has started!")

    def inference_callback(self, msg):
        # 1. Convert ROS Image to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]

        # 2. Preprocess exactly like DrivingDataset._preprocess
        #    - crop lower 60%
        top = int(h * 0.40)
        cropped = frame[top:, :]

        #    - resize to (out_w, out_h)
        resized = cv2.resize(
            cropped,
            (self.out_w, self.out_h),
            interpolation=cv2.INTER_AREA
        )

        #    - BGR -> RGB, normalize to [0,1]
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

        #    - HWC -> CHW
        chw = np.transpose(rgb, (2, 0, 1))  # (3, 66, 200)

        #    - numpy -> torch tensor, add batch dim
        x = torch.from_numpy(chw).unsqueeze(0).to(self.device)  # (1, 3, 66, 200)

        # 3. Run model
        with torch.no_grad():
            pred_norm = self.model(x).item()  # in [-1, 1] because of tanh

        # 4. Map normalized output to degrees (or whatever your servo expects)
        #    If your labels in CSV were already in [-1,1], this maps to ±max_steer_deg.
        steering_angle = float(pred_norm * self.max_steer_deg)

        # 5. Temporal smoothing
        self.steering_buffer.append(steering_angle)
        self.frame_count += 1

        # Log raw prediction
        self.get_logger().info(f"Raw model steering (deg): {steering_angle:.2f}")

        # 6. Publish smoothed steering angle every N frames
        if self.frame_count % self.publish_every_n_frames == 0:
            self.smoothed_angle = float(np.mean(self.steering_buffer))

            # Simple proportional control (tune Kp!)
            Kp = 1.8
            command_angle = Kp * self.smoothed_angle

            motor_msg = Motor()
            motor_msg.angle = int(command_angle)
            motor_msg.speed = int(15)  # tune as you like
            self.motor_pub.publish(motor_msg)

            self.get_logger().info(
                f"Publishing smoothed steering angle: {self.smoothed_angle:.2f} deg "
                f"(command: {command_angle:.2f})"
            )

        # 7. Draw text on the original frame (or on cropped if you prefer)
        combo = frame.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        position = (50, 100)
        font_scale = 1.5
        color = (0, 0, 255)
        thickness = 2
        line_type = cv2.LINE_AA

        cv2.putText(
            combo,
            f"Steer: {self.smoothed_angle:.2f} deg",
            position,
            font,
            font_scale,
            color,
            thickness,
            line_type,
        )

        # 8. Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(combo, "bgr8")
        self.image_pub.publish(annotated_msg)


def main():
    rclpy.init()
    node = LaneInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
