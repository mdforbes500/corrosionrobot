#!/usr/bin/env python

#----------------------------------------#
# @title: system_subscriber.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.2.1
#
# @licsense: MIT
#----------------------------------------#

import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import os

def callback1(data):
    br = CvBridge()
    rp.loginfo('Recieving front image')
    frame = br.imgmsg_to_cv2(data)
    width, height = cv.GetSize(frame)
    cv.line(frame,(0,0),(0,0),(255,255,255),2)
    cv.line(frame,(0,0),(511,511),(255,255,255),2)
    cv.imshow('Front Video Feed', frame)
    k = cv.waitKey(1)

    if k%256 == 27:
        # ESC key is pressed
        print("Terminating video feed...")
        cv.destroyWindow('Front Video Feed')
    elif k%256 == 32:
        # SPACE pressed
        print("Saving file...")
        img_name = input("Save as: ")
        path = os.path.join(os.path.expanduser("~"), "Pictures", img_name)
        cv.imwrite(path, frame)
        print("{0} saved at {1}".format(img_name, path))

def callback2(data):
    rp.loginfo('Recieving distance to end of pipe %s', data.data)

def system_listener():
    rp.init_node('listener', anonymous=False)
    rp.Subscriber('image_raw', Image, callback1)
    rp.Subscriber('distance', String, callback2)

    rp.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    system_listener()
