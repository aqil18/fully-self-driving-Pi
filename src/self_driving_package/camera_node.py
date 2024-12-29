#!/usr/bin/env python3

## Captures images and publishes them to a topic, e.g., /camera/image_raw.

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def camera_publisher():
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)  # Open default camera

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(image_msg)
        rate.sleep()

    cap.release()

if __name__ == "__main__":
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
