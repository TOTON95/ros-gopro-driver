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
from datetime import datetime

# Request process
def request_proc(URL):
    PARAMS = {}
    r = requests.get(url=URL, params=PARAMS)

# Shutter callback (shoot photo, start recording)
def cb_shutter(msg, URL):
    rospy.loginfo("Shutter activated\n")
    request_proc(URL)

# Stop signal callback (stop recording)
def cb_stop(msg, URL):
    rospy.loginfo("Recording stopped\n")
    request_proc(URL)

# Video mode signal callback (change to video record)
def cb_mode(msg, URLS):
    rospy.loginfo("Accessing: " + str(URLS[msg.data]))
    request_proc(URLS[msg.data])

# Get the date and time, convert it to gopro format
def set_date_time(URL):
    now = datetime.now()
    year = format(int(now.strftime("%y")), '02x')
    month = format(int(now.strftime("%m")), '02x')
    day = format(int(now.strftime("%d")), '02x')
    hour = format(int(now.strftime("%H")), '02x')
    minute = format(int(now.strftime("%M")), '02x')
    seconds = format(int(now.strftime("%S")), '02x')
    # rospy.loginfo("Hex date: "+year+" "+month+" "+day+" "+hour+" "+minute+" "+seconds)

    # Build the string for date time
    date_time = str("%"+year+"%"+month+"%"+day+"%"+hour+"%"+minute+"%"+seconds)
    rospy.loginfo("GoPro datetime: " + date_time)
    set_date_time_url = URL+date_time
    request_proc(set_date_time_url)

# Tag the moment on the video
def cb_tag(URL):
    rospy.loginfo("TAG recorded!")
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
            r = requests.get(url=URL, params=PARAMS)
            rospy.loginfo("Result: " + str(r.status_code))
            if(r.status_code == 200):
                msg = Empty()
                p_live.publish(msg)
            time.sleep(10)


# Init function
def init():
    rospy.init_node('gopro_node', anonymous=True)
    rospy.loginfo("Starting gopro node: " + rospy.get_name() + "...\n")

    try:
        param_list = rospy.get_param_names()
        rospy.loginfo(param_list)
    except:
        rospy.loginfo("Couldn't get any parameter names")

    # Getting Parameters
    shutter_url = rospy.get_param("trigger")
    stop_url = rospy.get_param("stop")
    live_url = rospy.get_param("wake_up_live")
    video_mode_url = rospy.get_param("video_mode")
    photo_mode_url = rospy.get_param("photo_mode")
    multi_shot_mode_url = rospy.get_param("multi_shot_mode")
    date_time_url = rospy.get_param("datetime")
    tag_url = rospy.get_param("tag_moment")

    # Packaging urls
    mode_urls = [video_mode_url, photo_mode_url, multi_shot_mode_url]

    # Setting up subscribers
    s_shutter = rospy.Subscriber('gp_shutter', Empty, cb_shutter, shutter_url)
    s_stop = rospy.Subscriber('gp_stop', Empty, cb_stop, stop_url)
    s_mode = rospy.Subscriber('gp_mode', UInt8, cb_mode, mode_urls)

    # Setup date and time
    set_date_time(date_time_url)

    # Suscriber for TAG Moments
    s_tag = rospy.Subscriber('gp_tag', Empty, cb_tag, tag_url)

    # Create thread for live stream
    gopro_mon = GOPRO_LIVE_MON(live_url)

    # "Refresh" ros node
    rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        print("GoPro -> Disconnected")
