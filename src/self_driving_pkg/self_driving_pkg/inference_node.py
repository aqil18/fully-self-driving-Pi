#!/usr/bin/env python3

## Subscribes to the camera topic, performs inference, and publishes commands to a motor topic, e.g., /motor/cmd.
## Aim: 
# 1. Draws bounding lines around detected lane in the image
# 2. Draws an angle of direction relative to the bounding lines and normal line

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        
        self.motor_pub = self.create_publisher(String, '/motor/cmd', 10)
        self.image_pub = self.create_publisher(Image, '/inference/image_out', 10)
        self.create_subscription(Image, '/camera/image_raw', self.inference_callback, 10)
        self.get_logger().info("Inference node has started!")


    def inference_callback(self, data):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

        # Perform inference (example: object detection)
        # Example: Draw bounding lines and calculate angle
        # (This is a placeholder for actual lane detection and angle calculation logic)
        cv2.line(frame, (100, 200), (200, 200), (0, 255, 0), 2)  # Example bounding line
        angle_of_direction = 30  # Example angle calculation
        self.get_logger().info(f"Angle of direction: {angle_of_direction} degrees")

        # Calculate difference from normal line (e.g., 0 degrees)
        angle_difference = angle_of_direction - 0
        command = f"Right {angle_difference}" if angle_difference > 0 else f"Left {abs(angle_difference)}"
        self.get_logger().info(f"Publishing command: {command}")
        self.motor_pub.publish(String(data=command))


        # Publish annotated frame
        annotated_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(annotated_msg)


def main():
    rclpy.init()
    node = InferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
