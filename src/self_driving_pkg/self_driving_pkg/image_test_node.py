#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePublisher(Node):
    def __init__(self, directory, topic_name, rate_hz):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()
        self.directory = directory
        self.rate_hz = rate_hz
        self.image_files = [f for f in os.listdir(directory) if f.endswith(('.png', '.jpg', '.jpeg'))]

        if not self.image_files:
            self.get_logger().error(f"No image files found in directory: {directory}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Publishing images from directory: {directory}")
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_image)
        self.image_index = 0

    def publish_image(self):
        if not rclpy.ok():
            return

        image_file = self.image_files[self.image_index]
        image_path = os.path.join(self.directory, image_file)
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().warn(f"Failed to read image: {image_path}")
            return

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(ros_image)
        self.get_logger().info(f"Published image: {image_file}")

        self.image_index = (self.image_index + 1) % len(self.image_files)


def main(args=None):
    rclpy.init(args=args)
    image_directory = '/example_images'  # Change this path to your image folder
    topic = '/camera/image_raw'
    publish_rate = 1  # Hz

    image_publisher = ImagePublisher(image_directory, topic, publish_rate)
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
