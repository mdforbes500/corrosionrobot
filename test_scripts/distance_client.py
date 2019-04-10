#!/usr/bin/env python

#----------------------------------------#
# @title: distance_client.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.1.0
#
# @licsense: MIT
# @description: Serves the distance of the ultrasonic sensor
#----------------------------------------#

import rospy as rp
from corrosion_robot.srv import *

def distance_client():
    rp.wait_for_service('get_distance')
    try:
        get_distance = rp.ServiceProxy('get_distance', SetSonar)
        distance = get_distance()
        return distance
    except rospy.ServiceException, e:
        print("Service call failed... {0}".format(e))

if __name__ == "__main__":
    distance = distance_client()
    rp.set_param('distance', distance)
