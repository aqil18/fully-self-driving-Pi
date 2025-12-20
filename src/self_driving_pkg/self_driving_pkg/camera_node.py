#!/usr/bin/env python3

## DEPRECATED: Use v4l2_camera_node instead and remove dependency on picamera2
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
from picamera2 import Picamera2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.bridge = CvBridge()
        
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.preview_configuration(main={"format": "BGR888", "size": (640, 480)}))
        self.picam2.start()
        
    def capture_and_publish(self):
        while rclpy.ok():
            frame = self.picam2.capture_array()
            # Draw a normal direction line in the center
            height, width, _ = frame.shape
            cv2.line(frame, (width // 2, height), (width // 2, height // 2), (0, 255, 0), 2)

            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(image_msg)
            self.get_logger().info("Published frame to /camera/image_raw")

            self.rate.sleep()

    def destroy(self):        
        self.picam2.stop()

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
