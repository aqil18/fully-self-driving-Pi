#!/usr/bin/env python3

## Captures images and publishes them to a topic, e.g., /camera/image_raw.

## Aim: 
# 1. Receives camera input from the Raspberry Pi camera
# 2. Draws a normal direction line on the image
# 3. Publishes the image to a topic, e.g., /camera/image_raw
# Input: RaspberryPi Video
# Output: Publish an image to the camera topic
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(0)  # Open default camera

    def capture_and_publish(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(image_msg)
            self.rate.sleep()

    def destroy(self):
        self.cap.release()

def main():
    rclpy.init()
    node = CameraNode()

    try:
        node.capture_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
