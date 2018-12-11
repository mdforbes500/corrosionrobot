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
from cv_bridge import CvBridge
import cv2 as cv

def callback1(data):
    br = CvBridge()
    rp.loginfo('recieving image')
    cv.imshow('camera_driver', br.imgmsg_to_cv2(data))
    cv.waitKey(5)

def callback2(data):
    rp.loginfo('recieving distance %s', data.data)

def system_listener():
    rp.init_node('listener', anonymous=False)
    rp.Subscriber('image_raw', Image, callback1)
    rp.Subscriber('distance', String, callback2)
    rp.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    system_listener()
