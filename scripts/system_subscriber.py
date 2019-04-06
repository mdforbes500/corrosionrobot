#!/usr/bin/env python

#----------------------------------------#
# @title: system_subscriber.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.1.0
#
# @licsense: MIT
#----------------------------------------#

import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv

def callback1(data):
    br = CvBridge()
    rp.loginfo('recieving front image')
    cv.imshow('camera_driver', br.imgmsg_to_cv2(data))
    cv.waitKey(5)

def callback2(data):
    br = CvBridge()
    rp.loginfo('recieving rear image')
    cv.imshow('camera_driver', br.imgmsg_to_cv2(data))
    cv.waitKey(5)

def callback3(data):
    rp.loginfo('recieving distance to end of pipe %s', data.data)

def callback4(data):
    rp.loginfo('recieving distance to front target %s', data.data)

def callback5(data):
    rp.loginfo('recieving distance to rear target %s', data.data)

def callback6(data):
    rp.loginfo('recieving distance from start of pipe %s', data.data)


def system_listener():
    rp.init_node('listener', anonymous=False)
    rp.Subscriber('image_raw', Image, callback1)
    rp.Subscriber('image_raw', Image, callback2)
    rp.Subscriber('distance', String, callback3)
    rp.Subscriber('distance', String, callback4)
    rp.Subscriber('distance', String, callback5)
    rp.Subscriber('distance', String, callback6)
    rp.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    system_listener()
