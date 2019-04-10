#!/usr/bin/env python

#----------------------------------------#
# @title: image_proc2.py
#
# @author: Joseph Hawkey & Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.2.1
#
# @licsense: MIT
# @description: When given an image of corrosion, returns the area of
# the corrosion.
#----------------------------------------#

from __future__ import division
from PIL import Image
from PIL import ImageFilter
import pylab as plt
import cv2 as cv
import numpy as np
import math
import sys

def main(args):
    #Inputs from system arguments
    filehandle = args[1]
    print("Opening and processing '{0}'...".format(filehandle))
    distance = float(args[2]) #cm
    print("Distance from object: {0} cm".format(distance))

    #Opening filehandle for reading and saving in memory as grayscale image
    image = cv.imread(filehandle)
    image_gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    ret, corrosion_thresh = cv.threshold(image_gray,127,255,0)
    ret2, coating_thresh = cv.threshold(image_gray, 127,255,0)
    cv.imshow('Gray image', image_gray)
    cv.waitKey(0)
    cv.imshow('Corrosion Threshold image', corrosion_thresh)
    cv.waitKey(0)
    cv.imshow('Coating Threshold image', coating_thresh)
    cv.waitKey(0)

    corrosion_contours,hierarchy = cv2.findContours(corrosion_thresh, 1, 2)
    coating_contours,hierarchy = cv2.findContours(coating_thresh, 1, 2)
    cnt = contours[0]

    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    im = cv2.drawContours(im,[box],0,(0,0,255),2)

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

main(sys.argv)
