#!/usr/bin/env python3


## TO DEPRECATE

## Aim:
# 1. Calculates the difference between the angle of direction and the normal line (Can be separated into angle node)
# 2. Publishes the difference as a command to the motor topic, e.g., /motor/cmd (Can be separated into angle node)

import rospy
from std_msgs.msg import String

def angle_callback(data):
    angle_of_direction = int(data.data)
    rospy.loginfo(f"Received angle of direction: {angle_of_direction} degrees")

    # Calculate difference from normal line (e.g., 0 degrees)
    angle_difference = angle_of_direction - 0
    command = f"Right {angle_difference}" if angle_difference > 0 else f"Left {abs(angle_difference)}"
    rospy.loginfo(f"Publishing command: {command}")
    motor_pub.publish(command)

def angle_node():
    global motor_pub
    rospy.init_node('angle_node', anonymous=True)
    rospy.Subscriber('/angle/direction', String, angle_callback)
    motor_pub = rospy.Publisher('/motor/cmd', String, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        angle_node()
    except rospy.ROSInterruptException:
        pass
