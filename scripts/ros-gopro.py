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

# Request process
def request_proc(URL):
    PARAMS = {}
    r = requests.get(url = URL, params = PARAMS)

# Shutter callback (shoot photo, start recording)
def cb_shutter(msg, URL):
    rospy.loginfo("Shutter activated\n" )
    request_proc(URL)

# Stop signal callback (stop recording)
def cb_stop(msg, URL):
    rospy.loginfo("Recording stopped\n")
    request_proc(URL)

# Request live signal from GOPRO
def cb_live(msg, URL):
    rospy.loginfo("Streaming...\n")
    rospy.loginfo("No streaming...\n")

# Init function
def init():
    rospy.init_node('gopro_node', anonymous=True)
    rospy.loginfo("Starting gopro node: " + rospy.get_name() + "...\n")
    # Getting Parameters
    shutter_url = rospy.get_param("trigger")
    print(shutter_url)
    stop_url = rospy.get_param("stop")
    live_url = rospy.get_param("wake_up_live")

    # Setting up subscribers
    s_shutter = rospy.Subscriber('gp_shutter', Empty, cb_shutter, shutter_url)
    s_stop = rospy.Subscriber('gp_stop', Empty, cb_stop, stop_url)
    s_live = rospy.Subscriber('gp_live', Empty, cb_live, live_url)

    # "Refresh" ros node
    rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        print("GoPro -> Disconnected")
