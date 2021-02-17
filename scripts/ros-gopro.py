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
import threading
import time

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

# Live-streaming monitor
class GOPRO_LIVE_MON(object):

    def __init__(self, url):
        thread = threading.Thread(target=self.run, args=(url,))
        thread.daemon = True
        thread.start()

    def run(self, URL):
        # Create publisher
        p_live = rospy.Publisher("gp_live", Empty, queue_size=10)

        while True:
            # Sending the request and saving response
            rospy.loginfo("Signal sent to " + URL)
            PARAMS = {}
            r = requests.get(url = URL, params = PARAMS)
            rospy.loginfo("Result: " + str(r.status_code))
            if(r.status_code == 200):
                msg = Empty()
                p_live.publish(msg)
            time.sleep(10)


# Init function
def init():
    rospy.init_node('gopro_node', anonymous=True)
    rospy.loginfo("Starting gopro node: " + rospy.get_name() + "...\n")
    # Getting Parameters
    shutter_url = rospy.get_param("trigger")
    stop_url = rospy.get_param("stop")
    live_url = rospy.get_param("wake_up_live")

    # Setting up subscribers
    s_shutter = rospy.Subscriber('gp_shutter', Empty, cb_shutter, shutter_url)
    s_stop = rospy.Subscriber('gp_stop', Empty, cb_stop, stop_url)

    # Create thread for live stream
    gopro_mon =  GOPRO_LIVE_MON(live_url)

    # "Refresh" ros node
    rospy.spin()

# Main function
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        print("GoPro -> Disconnected")
