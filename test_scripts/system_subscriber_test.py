#!/usr/bin/env python

#----------------------------------------#
# @title: system_subscriber.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.2.0
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
    frame = br.imgmsg_to_cv2(data)
    cv.imshow('camera_driver', frame)
    k = cv.waitKey(1)

    if k%256 == 27:
        # ESC key is pressed
        print("Escape pressed, closing...")
    elif k%256 == 32:
        # SPACE pressed
        img_name = "site_{}.png".format(0)
        cv.imwrite(img_name, frame)
        print("{} captured!".format(img_name))

def callback2(data):
    rp.loginfo('recieving distance to end of pipe %s', data.data)

def system_listener():
    rp.init_node('listener', anonymous=False)
    rp.Subscriber('image_raw', Image, callback1)
    rp.Subscriber('distance', String, callback2)

    rp.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    system_listener()
