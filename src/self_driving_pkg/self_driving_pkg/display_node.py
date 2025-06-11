# Aim
# 1. To display an output of the inference model for testing purposes.

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/inference/image_out', self.display_callback, 10)
        self.get_logger().info("Display node started.")

    def display_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Inference Output", frame)
        cv2.waitKey(1)  # Required to keep the window responsive

def main():
    rclpy.init()
    node = DisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
