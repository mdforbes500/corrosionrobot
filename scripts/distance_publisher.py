#!/usr/bin/env python

#----------------------------------------#
# @title: distance_publisher.py
#
# @author: Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.1.0
#
# @licsense: MIT
# @description: Publishes the distance of the ultrasonic sensor
#----------------------------------------#

import rospy as rp
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import uint8

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 18
GPIO=ECHO = 24

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance_publisher():
    GPIO.output(GPIO_TRIGGER, True)    
