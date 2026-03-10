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

from ml.pipilotnet import PiPilotNet
from ml.preprocessor import PreProcessor

from config import Config



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

        # Convolutional Neural Network - 
        # create an untrained model and then load the parameters onto it
        self.model = PiPilotNet()
        self.preprocessor = PreProcessor()
        # Parameters are stored in state_dict
        ckpt = torch.load(self.cfg.model_path, map_location=self.device)
        # Use weights_only=True for best practice when loading weights
        self.model.load_state_dict(ckpt["model_state"])
        # Set the model to evaluation mode
        self.model.eval()

        # RPI settings
        self.cfg = Config()
        
        self.get_logger().info("Lane inference node has started!")

    def inference_callback(self, msg):
        # 1. Convert ROS Image to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 2. Preprocess using the same preprocessor as used in the training
        x = self.preprocessor.preprocess(frame)

        # 3. Run model
        with torch.no_grad():
            # Giving the model my input and getting an output tensor
            pred_steering, pred_throttle = self.model(x)[0] 
            pred_steering = pred_steering.item()
            pred_throttle = pred_throttle.item()

        # 4. Map normalized output to degrees (or whatever your servo expects)
        #    If your labels in CSV were already in [-1,1], this maps to ±max_steer_deg.
        steering_angle = float(pred_steering * self.cfg.max_angle)
        throttle = float(pred_throttle * self.cfg.max_throttle)

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
            motor_msg.speed = int(throttle)
            self.motor_pub.publish(motor_msg)

            self.get_logger().info(
                f"Publishing smoothed steering angle: {self.smoothed_angle:.2f} deg "
                f"(command: {command_angle:.2f})"
            )

        # 7. Draw text on the original frame
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
