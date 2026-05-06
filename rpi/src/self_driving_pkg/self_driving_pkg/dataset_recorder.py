import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Motor as MotorMsg
from std_msgs.msg import Empty, Int32
from cv_bridge import CvBridge
import cv2
import csv
import os
import numpy as np
from datetime import datetime

class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')

        self.bridge = CvBridge()
        self.latest_cmd = None
        self._latest_offset_px: int = 0  # raw pixel offset from detection_node
        self.count = 0
        self.file_name = 'datasets/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        self.image_sub = self.create_subscription(
            Image, '/fsd/image_raw', self.image_callback, 10)

        self.cmd_sub = self.create_subscription(
            MotorMsg, '/motor/cmd', self.cmd_callback, 10)

        self.offset_sub = self.create_subscription(
            Int32, '/detection/offset', self.offset_callback, 10)

        self.shutdown_sub = self.create_subscription(
            Empty, '/teleop/shutdown', self.shutdown_callback, 10)

        os.makedirs(self.file_name + '/images', exist_ok=True)
        os.makedirs(self.file_name + '/labels', exist_ok=True)
        self.csv_file = open(self.file_name + '/labels/labels.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['filename', 'steering', 'throttle', 'offset'])

    def shutdown_callback(self, msg):
        self.get_logger().info("Teleop shutdown received, stopping dataset recorder.")
        rclpy.shutdown()

    def cmd_callback(self, msg: MotorMsg):
        self.latest_cmd = msg

    def offset_callback(self, msg: Int32):
        self._latest_offset_px = msg.data

    def image_callback(self, msg: Image):
        if self.latest_cmd is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Normalise offset using the original (full-res) frame width before resizing
        frame_w = frame.shape[1]
        offset_norm = float(np.clip(self._latest_offset_px / (frame_w / 2), -1.0, 1.0))

        img = cv2.resize(frame, (160, 120))
        image_name = f'{self.count:06d}.jpg'
        cv2.imwrite(f'{self.file_name}/images/{image_name}', img)

        steering = self.latest_cmd.angle
        throttle = self.latest_cmd.speed

        self.writer.writerow([image_name, steering, throttle, f'{offset_norm:.4f}'])
        self.get_logger().info(
            f"Recorded {image_name}  steering={steering}  throttle={throttle}  offset={offset_norm:.3f}"
        )
        self.count += 1
        
def main(args=None):
    rclpy.init(args=args)
    dataset_recorder = DatasetRecorder()
    rclpy.spin(dataset_recorder)
    dataset_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()