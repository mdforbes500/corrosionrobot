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
from std_msgs.msg import String
import sys

myargs = rp.myargv(argv=sys.argv)

GPIO.setmode(GPIO.BCM)

GPIO_TRIGGER=int(myargs[1])
GPIO_ECHO=int(myargs[2])

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance_publisher():
    pub = rp.Publisher('distance', String, queue_size=10)
    rp.init_node('ultrasonic', anonymous=True)
    rate = rp.Rate(10)   #10 Hz

    while not rp.is_shutdown():
        GPIO.output(GPIO_TRIGGER, True)
        rate.sleep()
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
        rp.loginfo(distance_str)
        pub.publish(distance_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        distance_publisher()
    except rp.ROSInterruptException:
        GPIO.cleanup()
