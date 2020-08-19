#!/usr/bin/env python3
'''
@file ros-gopro.py
@author Alexis Guijarro
@date August 2020
'''
# Importing libraries
import rospy
from std_msgs.msg import Empty, String, UInt8
import requests


def cb_shutter(msg):
    rospy.loginfo("Shutter activated\n")

def cb_stop(msg):
    rospy.loginfo("Recording stopped\n")

def cb_live(msg):
    rospy.loginfo("Streaming...\n")
    rospy.loginfo("No streaming...\n")


def init():
    rospy.init_node('gopro_node', anonymous=True)
    rospy.loginfo("Starting gopro node: " + rospy.get_name() + "...\n")
    s_shutter = rospy.Subscriber('gp_shutter', Empty, cb_shutter)
    s_stop = rospy.Subscriber('gp_stop', Empty, cb_stop)
    s_live = rospy.Subscriber('gp_live', Empty, cb_live)

    rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        print("GoPro -> Disconnected")
