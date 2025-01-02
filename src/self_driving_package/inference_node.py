#!/usr/bin/env python3


## Subscribes to the camera topic, performs inference, and publishes commands to a motor topic, e.g., /motor/cmd.
## Aim: 
# 1. Draws bounding lines around detected lane in the image from camera topic
# 2. Draws an angle of direction relative to the bounding lines and normal line publishes the image to a display topic
# 3. Calculates angle of direction and publishes the command to a motor topic
# Input: Image from camera node
# Output: Annotated image for angle node for display topic and angle of direction for motor topic

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


# Callback from camera topic
def inference_callback(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # Perform inference (example: object detection)
    # Example: Draw bounding lines and calculate angle
    # (This is a placeholder for actual lane detection and angle calculation logic)
    cv2.line(frame, (100, 200), (200, 200), (0, 255, 0), 2)  # Example bounding line
    angle_of_direction = 30  # Example angle calculation
    rospy.loginfo(f"Angle of direction: {angle_of_direction} degrees")

    # Calculate difference from normal line (e.g., 0 degrees)
    angle_difference = angle_of_direction - 0
    command = f"Right {angle_difference}" if angle_difference > 0 else f"Left {abs(angle_difference)}"
    rospy.loginfo(f"Publishing command: {command}")
    motor_pub.publish(command)


# Draw lane boundaries
def drawLane():
    pass

def drawAngledLine():
    pass

def calculateAngle():
    return 0

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
