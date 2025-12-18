#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json

class LaneInferenceNode(Node):
    def __init__(self):
        super().__init__('lane_inference_node')
        
        self.motor_pub = self.create_publisher(String, '/motor/cmd', 10)
        self.image_pub = self.create_publisher(Image, '/inference/image_out', 10)
        self.create_subscription(Image, '/camera/image_raw', self.inference_callback, 10)

        self.bridge = CvBridge()
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
        polygon = np.array([[
            (0, height),
            (0, height//2),
            (width, height//2),
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
        action = "move"
        steering_angle = self.compute_steering_angle(frame, lane_lines)
        speed = 50
        self.get_logger().info(f"Steering angle: {steering_angle:.2f} | Command: {action}")
        
        command = {
            "action": action,
            "steering_angle": steering_angle,
            "speed": speed
        }
        # Convert the dictionary to a JSON formatted string
        data = json.dumps(command)
        
        # This needs to change
        self.motor_pub.publish(data)

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
            y2 = int(height*0.6)
            x1 = int((y1 - intercept)/slope)
            x2 = int((y2 - intercept)/slope)
            lane_lines.append([x1,y1,x2,y2])
        if right_lines:
            slope, intercept = np.mean(right_lines, axis=0)
            y1 = height
            y2 = int(height*0.6)
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
        angle = math.degrees(math.atan2(dx, height))
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
