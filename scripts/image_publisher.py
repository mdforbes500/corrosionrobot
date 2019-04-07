#!/usr/bin/env python

#----------------------------------------#
# @title: image_publisher.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.1.0
#
# @licsense: MIT
#----------------------------------------#

####################NEW TRIAL CODE############################

import rospy as rp
import imutils
import numpy as np
import argparse
from imutils.video import FileVideoStream
from imutils.video import FPS
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import sys

myargs = rp.myargv(argv=sys.argv)

def cv_publisher(index=myargs[1]):
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-v","--video",required=True,help="path to input video file")
    # args = vars(ap.parse_args())
    fvs = FileVideoStream(int(index)).start()
    time.sleep(1)
    pub = rp.Publisher('image_raw', Image, queue_size=10)
    rp.init_node('camera_driver', anonymous=False)
    rate = rp.Rate(1) #60 Hz
    br = CvBridge()
    while not rp.is_shutdown():
        while fvs.more():
            frame = fvs.read()
            # frame = imutils.resize(frame, width=450)
            # frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            frame = np.dstack([frame,frame,frame])
            rp.loginfo('publish image')
            pub.publish(br.cv2_to_imgmsg(frame))
            rate.sleep()

if __name__ == '__main__':
    try:
        cv_publisher()
    except rp.ROSInterruptException:
        pass











######################OLD CODE FOR IMAGE_PUBLISHER.PY##############################

# import rospy as rp
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2 as cv
# import sys
#
# myargs = rp.myargv(argv=sys.argv)
#
# def cv_publisher(index=myargs[1]):
#     pub = rp.Publisher('image_raw', Image, queue_size=10)
#     rp.init_node('camera_driver', anonymous=False)
#     rate = rp.Rate(1) #60 Hz
#     cap = cv.VideoCapture(int(index))
#     br = CvBridge()
#
#     while not rp.is_shutdown():
#         [ret, frame] = cap.read()
#         if ret == True:
#             rp.loginfo('publish image')
#             pub.publish(br.cv2_to_imgmsg(frame))
#
#         rate.sleep()
#
# if __name__ == '__main__':
#     try:
#         cv_publisher()
#     except rp.ROSInterruptException:
#         pass
