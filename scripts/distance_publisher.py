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
from std_msgs.msg import uint16

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 18
GPIO=ECHO = 24

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance_publisher():
    pub = rp.Publisher('distance', uint16, queue_size=10)
    rospy.init_node('ultrasonic', anonymous=False)
    rate = rospy.Rate(10)   #10 Hz

    while not rospy.is_shutdown():
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        start_time = rospy.get_time()
        stop_time = rospy.get_time()

        while GPIO.input(GPIO_ECHO) == 0:
            start_time = rospy.get_time()

        while GPIO.input(GPIO_ECHO) == 1:
            stop_time = rospy.get_time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300)/2
        distance_str = "%s" % distance
        rospy.loginfo(distance_str)
        pub.publish(distance_str)
        rate.sleep()

if __name__ = '__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        pass
