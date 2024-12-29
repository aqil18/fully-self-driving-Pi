#!/usr/bin/env python3

## Subscribes to the motor topic and acts on the commands.

## Aim:
# 1. Receives angle commands from the motor topic, e.g., /motor/cmd
# 2. Acts on the commands, e.g., Right 23 degrees, Left 43 degrees

import rospy
from std_msgs.msg import String

def motor_callback(data):
    command = data.data
    rospy.loginfo(f"Received motor command: {command}")
    # Example motor control logic:
    if command == "stop":
        rospy.loginfo("Stopping motor")
    elif command == "forward":
        rospy.loginfo("Moving forward")
    # Add more actions as needed

def motor_action_node():
    rospy.init_node('motor_action_node', anonymous=True)
    rospy.Subscriber('/motor/cmd', String, motor_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        motor_action_node()
    except rospy.ROSInterruptException:
        pass
