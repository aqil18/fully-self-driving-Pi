import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Motor as MotorMsg
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime

class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')

        self.bridge = CvBridge()
        self.latest_cmd = None
        self.count = 0
        self.file_name = 'datasets/' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') 
        self.image_sub = self.create_subscription(
            Image, '/fsd/image_raw', self.image_callback, 10)

        self.cmd_sub = self.create_subscription(
            MotorMsg, '/motor/cmd', self.cmd_callback, 10)

        os.makedirs(self.file_name + '/images', exist_ok=True)
        os.makedirs(self.file_name + '/labels', exist_ok=True)
        self.csv_file = open(self.file_name + '/labels/labels.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['filename', 'steering', 'throttle'])

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def image_callback(self, msg):
        if self.latest_cmd is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv2.resize(img, (160, 120))

        image_name = f'{self.count:06d}.jpg'
        cv2.imwrite(f'{self.file_name}/images/{image_name}', img)

        steering = self.latest_cmd.angle
        throttle = self.latest_cmd.speed

        self.writer.writerow([image_name, steering, throttle])
        self.get_logger().info(f"Recorded image: {image_name} with steering: {steering}, throttle: {throttle}")
        self.count += 1
        
def main(args=None):
    rclpy.init(args=args)
    dataset_recorder = DatasetRecorder()
    rclpy.spin(dataset_recorder)
    dataset_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()