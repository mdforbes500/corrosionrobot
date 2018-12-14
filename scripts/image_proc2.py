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

from sensor_msgs.msg import Image as img
from PIL import Image
import sys
from PIL import ImageFilter

def image_proc2(args):
	image = args[1]
	pic = Image.open(image, 'r')
	pixel_number_wide = pic.size[0]
	image_width = (1.50+1.70)/3.04*(3280/3.68)*(680/3280)*float(args[2])

	pixel_size = (image_width/float(pixel_number_wide))**2

	print("Pixel size: {0} ".format(pixel_size))
	pixels = list(pic.getdata())
	counter = 0
	corCount = 0

	for x in pixels:
		if(x[0]+x[1]+x[2] >= 0):
			counter+=1
		if(x[0]+x[1]+x[2] < 300):
			corCount+=1

	print("Number of pixels: {0}\n".format(counter))
	print("Number of corrosion pixels: {0}\n".format(corCount))
	corrosion_area = pixel_size*corCount
	print("Total Area of Corrosion: {0} Square Centimeters".format(corrosion_area))

if __name__ == "main":
    image_proc2(sys.argv)
