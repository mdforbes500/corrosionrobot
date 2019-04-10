#! /usr/bin/env python
from __future__ import division
from PIL import Image
from PIL import ImageFilter
import pylab as plt
import cv2 as cv
import numpy as np
import math
import sys
import rospy
import FindArea.srv

def find_area():
    #Inputs from system arguments
    filehandle = args[1]
    print("Opening and processing '{0}'...".format(filehandle))
    distance = float(args[2]) #cm

    #Opening filehandle for reading and saving in memory as grayscale image
    image = cv.imread(filehandle)
    image_gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    cv.imshow('Gray image', image_gray)
    cv.waitKey(0)

    # Determining the CMOS width and height in pixels
    width = np.size(image_gray, 1) #px
    height = np.size(image_gray, 0) #px

    # Field of View for PiCamera V2
    FOV = (62.2, 48.8) #deg: horizontal X vertical

    #Angle of one pixel for both height and width
    w_angle = 1/width*FOV[0] #deg
    h_angle = 1/height*FOV[1] #deg

    #Width and Height of one pixel in cm
    px_width = distance*math.tan(math.radians(w_angle)) #cm
    px_height = distance*math.tan(math.radians(h_angle)) #cm

    #Image width and height in cm
    image_width = px_width*width #cm
    image_height = px_height*height #cm

    #Area of a single pixel
    pixel_area = px_width*px_height #cm^2

    #Counting total number of corroded pixels
    deviation = int(np.std(image))
    counter = width*height
    CountPixelB = 0
    CountPixelW = 0
    for y in range(height-1):
        for x in range(width-1):
            if image_gray[y,x] >= 127:
                CountPixelW += 1
            if image_gray[y,x] < 127:
                CountPixelB += 1

    #Total corroded area in frame
    corrosion_area = pixel_area*CountPixelB
    print("Total Area of Corrosion: {0:.{1}f} cm^2".format(corrosion_area,3))

if __name__ == "__main__":
    find_area()
