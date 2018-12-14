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

from PIL import Image
import sys
from PIL import ImageFilter

def main(args):
    image = args[1]
    pic = Image.open(image, 'r')
    width = float(pic.size[0]) #px
    sensor_size = 0.386 #cm
    focal_length = 0.304 #cm
    distance = float(args[2]) #cm
    image_width = sensor_size*distance/focal_length*640/2464 #cm
    print("Reading '{0}'...".format(args[1]))
    print("Distance from object: {0} cm".format(args[2]))
    print("Image width: {0:.{1}f} cm".format(image_width,3))

    pixel_area = (image_width/width)**2
    pixels = list(pic.getdata())
    counter = 0
    corCount = 0

    for x in pixels:
        if(x[0]+x[1]+x[2] >= 0):
            counter+=1
        if(x[0]+x[1]+x[2] < 300):
            corCount+=1

    print("Number of pixels: {0:d} pixels".format(counter))
    print("Number of corrosion pixels: {0:d} pixels".format(corCount))
    corrosion_area = pixel_area*corCount
    print("Total Area of Corrosion: {0:.{1}f} cm^2".format(corrosion_area,3))


main(sys.argv)
