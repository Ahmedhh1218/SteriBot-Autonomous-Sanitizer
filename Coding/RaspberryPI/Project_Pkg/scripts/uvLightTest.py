#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8
import RPi.GPIO as GPIO

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pin as output
led_pin = 16
GPIO.setup(led_pin, GPIO.OUT)

# Callback function for ROS messages
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)
    if data.data == 1:
        GPIO.output(led_pin, GPIO.HIGH)
    elif data.data == 0:
        GPIO.output(led_pin, GPIO.LOW)

def listener():
    # Initialize ROS node
    rospy.init_node('led_control', anonymous=True)

    # Subscribe to ROS topic
    rospy.Subscriber("uv_control_topic", UInt8, callback)

    # Spin to keep the node from exiting
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    finally:
        # Clean up GPIO settings
        GPIO.cleanup()
