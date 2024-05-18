#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def flag_callback(msg):
    flag = msg.data
    if flag == 1:
        print("Obstacle detected in front")
    elif flag == 2:
        print("Obstacle detected on the left")
    elif flag == 3:
        print("Obstacle detected on the right")

def ir_subscriber():
    rospy.init_node('ir_subscriber', anonymous=True)
    rospy.Subscriber('/ir_flag', Float64, flag_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ir_subscriber()
    except rospy.ROSInterruptException:
        pass
