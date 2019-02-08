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
import numpy as np
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

    #Opening filehandle for reading and saving in memory as grayscale image
    image = np.array(Image.open(filehandle, 'r').convert('L'),'f')

    # Determining the CMOS width and height in pixels
    width = image.shape[1] #px
    print("Width of image is : {} pixels".format(width))
    height = image.shape[0] #px
    print("Height of image is : {} pixels".format(height))

    # Field of View for PiCamera V2
    FOV = (62.2, 48.8) #deg: horizontal X vertical
    print("Horizontal field of view is {} degrees".format(FOV[0]))
    print("Vertical field of view is {} degrees".format(FOV[1]))

    #Angle of one pixel for both height and width
    w_angle = 1/width*FOV[0] #deg
    print("Width angle for a single pixel is {0:.4f} degrees".format(w_angle))
    h_angle = 1/height*FOV[1] #deg
    print("Height angle for a single pixel is {0:.4f} degrees".format(h_angle))

    #Width and Height of one pixel in cm
    px_width = distance*math.tan(math.radians(w_angle)) #cm
    print("Width of a single pixel is {0:.4f} cm".format(px_width))
    px_height = distance*math.tan(math.radians(h_angle)) #cm
    print("Height of a single pixel is {0:.4f} cm".format(px_height))

    #Image width and height in cm
    image_width = px_width*width #cm
    print("Image width: {0:.{1}f} cm".format(image_width,3))
    image_height = px_height*height #cm
    print("Image height: {0:.{1}f} cm".format(image_height,3))

    #Area of a single pixel
    pixel_area = px_width*px_height #cm^2
    print("Area of a single pixel: {0:.2e} cm^2".format(pixel_area))

    #Counting total number of corroded pixels
    deviation = int(np.std(image))
    counter = width*height
    black = 0
    for row in image:
        for pixel in row:
            if pixel >= 3*deviation:
                black += 1

    #Total corroded area in frame
    corrosion_area = pixel_area*black

    print("Number of pixels: {0:d} pixels".format(counter))
    print("Number of corrosion pixels: {0:d} pixels".format(black))
    print("Total Area of Corrosion: {0:.{1}f} cm^2".format(corrosion_area,3))

main(sys.argv)
