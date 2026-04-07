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

from .ml.pipilotnet import PiPilotNet
from .ml.preprocessor import PreProcessor

from .config import Config



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


        # RPI settings
        self.cfg = Config()
        
        # Convolutional Neural Network - 
        # create an untrained model and then load the parameters onto it
        self.model = PiPilotNet()
        self.preprocessor = PreProcessor()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        # Parameters are stored in state_dict
        ckpt = torch.load(self.cfg.model_path, map_location=self.device)
        # Use weights_only=True for best practice when loading weights
        self.model.load_state_dict(ckpt["model_state"])
        # Set the model to evaluation mode
        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info("Lane inference node has started!")

    def inference_callback(self, msg):
        # 1. Convert ROS Image to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 2. Preprocess using the same preprocessor as used in the training
        x = self.preprocessor.preprocess(frame)

        # 3. Run model
        x = torch.from_numpy(x).permute(2, 0, 1).unsqueeze(0).float().to(self.device)
        with torch.no_grad():
            # Giving the model my input and getting an output tensor
            pred_steering, pred_throttle = self.model(x)

        # 4. Map normalized output to degrees (or whatever your servo expects)
        #    If your labels in CSV were already in [-1,1], this maps to ±max_steer_deg.
        steering_angle = float(pred_steering * self.cfg.max_angle)
        throttle = float(pred_throttle * self.cfg.max_throttle)

        # 5. Temporal smoothing
        self.steering_buffer.append(steering_angle)
        self.frame_count += 1

        # 6. Publish smoothed steering angle every N frames
        if self.frame_count % self.publish_every_n_frames == 0:
            self.smoothed_angle = float(np.mean(self.steering_buffer))

            # # Simple proportional control (tune Kp!)
            # Kp = 1.8
            # command_angle = Kp * self.smoothed_angle

            motor_msg = Motor()
            motor_msg.angle = int(self.smoothed_angle)
            motor_msg.speed = int(throttle)
            self.motor_pub.publish(motor_msg)

            self.get_logger().info(
                f"steering={int(self.smoothed_angle)}  throttle={int(throttle)}"
            )

        # 7. Draw text on the original frame
        combo = frame.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 0, 255)
        line_type = cv2.LINE_AA

        cv2.putText(combo, f"Steer: {self.smoothed_angle:.2f} deg",
                    (10, 20), font, 0.5, color, 1, line_type)
        cv2.putText(combo, f"Throttle: {throttle:.2f}",
                    (10, 40), font, 0.5, color, 1, line_type)

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
