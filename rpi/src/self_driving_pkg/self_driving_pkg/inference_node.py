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

        self.get_logger().info("Lane inference node has started!")

    def inference_callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width = frame.shape[:2]

        # 1. Convert to grayscale and blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        # 2. Edge detection
        edges = cv2.Canny(blur, 50, 150)

        # 3. Region of interest (bottom half of the image)
        mask = np.zeros_like(edges)
        roi_top = int(height * 0.35)
        polygon = np.array([[
            (0, height),
            (0, roi_top),
            (width, roi_top),
            (width, height)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        # 4. Hough Transform to detect lines
        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=50)

        lane_lines = self.average_slope_intercept(frame, lines)

        # 5. Draw lane lines
        lane_frame = np.zeros_like(frame)
        for line in lane_lines:
            x1, y1, x2, y2 = line
            cv2.line(lane_frame, (x1,y1), (x2,y2), (0,255,0), 5)

        combo = cv2.addWeighted(frame, 0.8, lane_frame, 1, 1)

        # 6. Calculate steering angle
        steering_angle = self.compute_steering_angle(frame, lane_lines)
        
        
        # Store steering angle for smoothing
        self.steering_buffer.append(steering_angle)
        self.frame_count += 1

        # Define text parameters
        font = cv2.FONT_HERSHEY_SIMPLEX
        position = (50, 100) # Bottom-left corner of the text
        font_scale = 1.5
        color = (0, 0, 255) # Red color in BGR
        thickness = 2
        line_type = cv2.LINE_AA # For smoother text
        
    

        self.get_logger().info(f"Steering angle: {steering_angle:.2f}")

        #  Publish smoothed steering angle every n frames
        if self.frame_count % self.publish_every_n_frames == 0:
            self.smoothed_angle = float(np.mean(self.steering_buffer))
            Kp = 1.8  # start here, tune
            command_angle = Kp * self.smoothed_angle


            msg = Motor()
            msg.angle = int(command_angle)
            msg.speed = int(15)
            self.motor_pub.publish(msg)

            self.get_logger().info(
                f"Publishing smoothed steering angle: {self.smoothed_angle:.2f}"
            )

        cv2.putText(combo, f"Steering angle: {self.smoothed_angle:.2f}", position, font, font_scale, color, thickness, line_type)

        # 7. Publish annotated frame
        annotated_msg = self.bridge.cv2_to_imgmsg(combo, "bgr8")
        self.image_pub.publish(annotated_msg)

    # Helper functions
    def average_slope_intercept(self, frame, lines):
        left_lines = []
        right_lines = []

        if lines is None:
            return []

        for line in lines:
            for x1,y1,x2,y2 in line:
                if x2 - x1 == 0:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope*x1
                if slope < -0.3:
                    left_lines.append((slope, intercept))
                elif slope > 0.3:
                    right_lines.append((slope, intercept))

        lane_lines = []
        height = frame.shape[0]
        if left_lines:
            slope, intercept = np.mean(left_lines, axis=0)
            y1 = height
            y2 = int(height*0.35)
            x1 = int((y1 - intercept)/slope)
            x2 = int((y2 - intercept)/slope)
            lane_lines.append([x1,y1,x2,y2])
        if right_lines:
            slope, intercept = np.mean(right_lines, axis=0)
            y1 = height
            y2 = int(height*0.35)
            x1 = int((y1 - intercept)/slope)
            x2 = int((y2 - intercept)/slope)
            lane_lines.append([x1,y1,x2,y2])

        return lane_lines

    def compute_steering_angle(self, frame, lane_lines):
        height, width, _ = frame.shape
        if not lane_lines:
            return 0.0

        # Calculate midpoint between lane lines at bottom
        x_bottom = []
        for x1,y1,x2,y2 in lane_lines:
            x_bottom.append(x1)
            x_bottom.append(x2)
        lane_center = np.mean(x_bottom)

        # Image center
        center = width / 2
        dx = lane_center - center

        # Approximate steering angle
        # Positive dx → turn right
        angle = -math.degrees(math.atan2(dx, height))
        return angle

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
