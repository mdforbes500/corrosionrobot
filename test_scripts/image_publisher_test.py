#!/usr/bin/env python

#----------------------------------------#
# @title: image_publisher.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.1.1
#
# @licsense: MIT
#----------------------------------------#

import rospy as rp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import sys

def cv_publisher():
    pub = rp.Publisher('image_raw', Image, queue_size=10)
    rp.init_node('camera_driver', anonymous=False)
    rate = rp.Rate(1) #60 Hz
    cap = cv.VideoCapture(-1)
    br = CvBridge()

    img_counter = 0

    while not rp.is_shutdown():
        [ret, frame] = cap.read()
        if ret == True:
            rp.loginfo('publish image')
            pub.publish(br.cv2_to_imgmsg(frame))

        rate.sleep()

if __name__ == '__main__':
    try:
        cv_publisher()
    except rp.ROSInterruptException:
        pass
