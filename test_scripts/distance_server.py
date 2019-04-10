#!/usr/bin/env python

#----------------------------------------#
# @title: distance_service.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.2.0
#
# @licsense: MIT
# @description: Serves the distance of the ultrasonic sensor
#----------------------------------------#

import rospy as rp
import RPi.GPIO as GPIO
from corrosion_robot.srv import *
from std_msgs.msg import String
import sys

myargs = rp.myargv(argv=sys.argv)

GPIO.setmode(GPIO.BCM)

GPIO_TRIGGER = int(myargs[1])
GPIO_ECHO = int(myargs[2])

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def handle_get_distance():
    GPIO.output(GPIO_TRIGGER, True)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = rp.get_time()
    stop_time = rp.get_time()

    while GPIO.input(GPIO_ECHO) == 0:
        start_time = rp.get_time()

    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = rp.get_time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300)/2
    distance_str = "%s" % distance
    return distance_str

def distance_server():
    rp.init_node('ultrasonic')
    serv = rp.Service('get_distance', SetSonar, handle_get_distance)
    print("Ready to find distance...")
    rospy.spin()

if __name__ == '__main__':
    try:
        distance_server()
    except rp.ROSInterruptException:
        GPIO.cleanup()
