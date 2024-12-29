#!/usr/bin/env python3


## Subscribes to the camera topic, performs inference, and publishes commands to a motor topic, e.g., /motor/cmd.

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

def inference_callback(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # Perform inference (example: object detection)
    command = "stop"  # Example output after inference
    rospy.loginfo(f"Inference result: {command}")
    motor_pub.publish(command)

def inference_node():
    global motor_pub
    rospy.init_node('inference_node', anonymous=True)
    rospy.Subscriber('/camera/image_raw', Image, inference_callback)
    motor_pub = rospy.Publisher('/motor/cmd', String, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        inference_node()
    except rospy.ROSInterruptException:
        pass
