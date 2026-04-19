#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

ROI_TOP_FRAC = 0.3  # keep bottom 70% of frame


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.offset_pub = self.create_publisher(Int32, '/detection/offset', 10)
        self.image_pub  = self.create_publisher(Image,  '/detection/image_out', 10)
        self.create_subscription(Image, '/fsd/image_raw', self.detection_callback, 10)

        self.bridge = CvBridge()
        self.get_logger().info("Lane detection node started.")

    # ------------------------------------------------------------------
    def detection_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w  = frame.shape[:2]
        roi_top = int(h * ROI_TOP_FRAC)
        roi     = frame[roi_top:, :]

        # Threshold to isolate tape
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        out = frame.copy()
        # ROI line + centre line
        cv2.line(out, (0, roi_top), (w, roi_top), (0, 255, 255), 1)
        frame_center = w // 2
        cv2.line(out, (frame_center, roi_top), (frame_center, h), (0, 255, 255), 1)

        if not contours:
            cv2.putText(out, "NO LINE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
            self.get_logger().warn("No line detected.")
            return

        # All contours (green), largest (blue)
        shifted = [c + np.array([0, roi_top]) for c in contours]
        cv2.drawContours(out, shifted, -1, (0, 200, 0), 1)

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            self.get_logger().warn("Zero contour moment.")
            return

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) + roi_top

        largest_shifted = largest + np.array([0, roi_top])
        cv2.drawContours(out, [largest_shifted], -1, (200, 100, 0), 2)
        cv2.circle(out, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(out, (frame_center, cy), (cx, cy), (255, 0, 255), 2)

        offset = cx - frame_center
        cv2.putText(out, f"offset: {offset:+d} px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Publish offset
        offset_msg = Int32()
        offset_msg.data = offset
        self.offset_pub.publish(offset_msg)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        self.get_logger().info(f"offset={offset:+d}")


def main():
    rclpy.init()
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
