#!/usr/bin/env python

#----------------------------------------#
# @title: image_proc2.py
#
# @author: Joseph Hawkey & Malcolm D. Forbes
# @email: gmucorrosion@gmail.com
# @version: v.0.1.1
#
# @licsense: MIT
# @description: When given an image of corrosion, returns the area of
# the corrosion.
#----------------------------------------#

from __future__ import division
from PIL import Image
from PIL import ImageFilter
import math
import sys

def main(args):
    """
    Takes the filehandle of the image and the distance supplied by the user and
    calculates the area of the corroded site.
    """
    #Inputs from system arguments
    filehandle = args[1]
    print("Opening and processing '{0}'...".format(filehandle))
    distance = float(args[2]) #cm
    print("Distance from object: {0} cm".format(distance))

    #Opening filehandle for reading and saving in memory as image
    image = Image.open(filehandle, 'r')

    # Determining the image width and height in pixels
    width = image.size[0] #px
    height = image.size[1] #px
    print(width)

    # Field of View for PiCamera V2
    FOV = (62.2, 48.8) #deg: horizontal X vertical
    print(FOV[0])

    #Angle of one pixel for both height and width
    w_angle = 1/width*FOV[0] #deg
    h_angle = 1/height*FOV[1] #deg
    print(w_angle)

    #Width and Height of one pixel in cm
    px_width = distance*math.sin(math.radians(w_angle)) #cm
    px_height = distance*math.sin(math.radians(h_angle)) #cm

    #Image width and height in cm
    image_width = px_width*width #cm
    print("Image width: {0:.{1}f} cm".format(image_width,3))
    image_height = px_height*height #cm
    print("Image height: {0:.{1}f} cm".format(image_height,3))

    #Area of a single pixel
    pixel_area = px_width*px_height #cm^2

    #Setting RGB values in array
    pixels = list(image.getdata())

    #Counting total number of corroded pixels
    counter = 0
    corroded = 0
    for x in pixels:
        if(x[0]+x[1]+x[2] >= 0):
            counter+=1
        if(x[0]+x[1]+x[2] < 300):
            corroded+=1

    #Total corroded area in frame
    corrosion_area = pixel_area*corroded

    print("Number of pixels: {0:d} pixels".format(counter))
    print("Number of corrosion pixels: {0:d} pixels".format(corroded))
    print("Total Area of Corrosion: {0:.{1}f} cm^2".format(corrosion_area,3))

main(sys.argv)
