#!/usr/bin/env python

#----------------------------------------#
# @title: length_computation.py
#
# @author: Malcolm D. Forbes & Joseph Hawkey
# @email: gmucorrosion@gmail.com
# @version: v.0.3.4
#
# @licsense: MIT
# @description: When given an image of corrosion, filters it for corrosion,
# and determines the width and height of the corrosive area, as well as
# the centroid.
#----------------------------------------#

from __future__ import division
from skimage import measure
import cv2 as cv
import numpy as np
import math
import imutils
import os
import sys

def compute_length(image, w, h, s, d, z, dist, img):
    # Field of View for PiCamera V2
    FOV = (62.2, 48.8) #deg: horizontal X vertical

    width = np.size(image, 1) #px
    height = np.size(image, 0) #px

    #Angle of one pixel for both height and width
    w_angle = 1/width*FOV[0] #deg
    h_angle = 1/height*FOV[1] #deg

    #Width and Height of one pixel in cm
    px_width = d*math.tan(math.radians(w_angle)) #cm
    px_height = d*math.tan(math.radians(h_angle)) #cm

    #Computation of width and height of anamoly in inches
    real_width = px_width*w/2.54
    real_height = px_height*h/2.54

    #Ouptut the offset distance in inches
    displacement = (px_width*s)
    print(displacement)

    #Computation for all cases
    d_utc = 2.8 #cm
    d_r = 91.44 #cm
    d_bp = 152.87752 #cm
    d_lp = 194.05092 #cm
    d_b = 11.43 #cm
    if dist == "back" and img == "back":
        displacement = (z + d_utc - d_b + displacement)/2.54 #in
    elif dist == "back" and img == "front":
        displacement = (z - d_b + d_r - d_utc + displacement)/2.54 #in
    elif dist == "front" and img == "back":
        displacement = (d_lp - z - d_r + d_utc + d_b + displacement)/2.54 #in
    elif dist == "front" and img == "front":
        displacement = (d_lp - z + d_utc + d_b + displacement)/2.54 #in
    else:
        print("Invalid sensor positions. Exiting...")
        sys.exit(3)

    return real_width, real_height, displacement

def connected_component(image, raw_image):
    #corrosion_thresh = cv.bitwise_not(corrosion_thresh) #reverse colors

    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    labels = measure.label(image, neighbors=8, background=0)
    mask = np.zeros(image.shape, dtype="uint8")

    # loop over the unique components
    for label in np.unique(labels):
        #if this is the background label, ignore it
        if label == 0:
            continue
        # otherwise, construct the label mask and count the
        # number of pixels
        label_mask = np.zeros(image.shape, dtype="uint8")
        label_mask[labels == label] = 255
        num_pixels = cv.countNonZero(label_mask)

        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if num_pixels > 50:
            mask = cv.add(mask, label_mask)

    if (cv.__version__ == "3.3.0"):
        contours= cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    else:
        contours= cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    cnts = imutils.grab_contours(contours)
    #cnts = contours.sort_contours(cnts)[0]
    # loop over the contours
    characteristics = []
    for (i, cnt) in enumerate(cnts):
        # plot bounding boxes
        (x, y, w, h) = cv.boundingRect(cnt)
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        im = cv.drawContours(raw_image,[box],0,(0,0,255),2)

        # compute the center of the contour area and draw a circle
        # representing the center
        M = cv.moments(cnt)
        cX = int(M["m10"] / M["m00"])
        d = 480/2-cX
        cY = int(M["m01"] / M["m00"])

        # add width and height to characteristics
        characteristics.append([w,h,d])

        # draw the countour number on the image
        cv.putText(raw_image, "#{}".format(i), (cX - 20, cY), cv.FONT_HERSHEY_SIMPLEX,
		1.0, (0, 255, 0), 2)

    cv.imshow('Corrosion Contours', im)
    cv.waitKey(0)
    cv.destroyWindow('Corrosion Contours')

    return characteristics

def gray_filter(image, thresh, inverse):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (11, 11), 0)
    # threshold the image to reveal light regions in the
    # blurred image
    if inverse == 'black':
        thresh = cv.threshold(blurred, thresh, 255, cv.THRESH_BINARY_INV)[1]
    else:
        thresh = cv.threshold(blurred, thresh, 255, cv.THRESH_BINARY)[1]
    # perform a series of erosions and dilations to remove
    # any small blobs of noise from the thresholded image
    thresh = cv.erode(thresh, None, iterations=2)
    thresh = cv.dilate(thresh, None, iterations=4)

    return thresh

def YCrCb_filter(image, channel='cb', thresh=127):
    #Conversion from Blue Green Red to
    #Luminance, Red Correction, Blue Correction
    image_ycb = cv.cvtColor(image, cv.COLOR_BGR2YCrCb)
    cv.imshow("YCrCb Colorspace", image_ycb)
    cv.waitKey(0)
    cv.destroyWindow("YCrCb Colorspace")

    #Split colorspace into three channels
    y,cr,cb = cv.split(image_ycb)

    #Apply Gaussian Blur to each channel
    if channel == 'Y':
    #Y: Luminance channel
        y_blur = cv.GaussianBlur(y, (11, 11), 0)
        output = y_blur
        cv.imshow("Luminance Blurred", y_blur)
        cv.waitKey(0)
        cv.destroyWindow("Luminance Blurred")
    elif channel == 'Cb':
    #Cb: Blue Corrected
        cb_blur = cv.GaussianBlur(cb, (11, 11), 0)
        output = cb_blur
        cv.imshow("Red Correction Blurred", cb_blur)
        cv.waitKey(0)
        cv.destroyWindow("Red Correction Blurred")
    elif channel == 'Cr':
    #Cr: Red Corrected
        cr_blur = cv.GaussianBlur(cr, (11, 11), 0)
        output = cr_blur
        cv.imshow("Blue Correction Blurred", cr_blur)
        cv.waitKey(0)
        cv.destroyWindow("Blue Correction Blurred")
    else:
        print("Please select valid channel")
        return None

    #Threshold to remove light sections
    ret,threshold = cv.threshold(output,thresh,255,0)
    threshold = cv.erode(threshold, None, iterations=2)
    threshold = cv.dilate(threshold, None, iterations=4)

    return threshold

def write_output(filehandle, num, o_clock, z, length, width, depth):
    file = open(filehandle, "a+")
    file.write("Anamoly {0}\n".format(num))
    file.write("Corrosion site:\n")
    file.write("O'clock position: {0}'clock\n".format(o_clock))
    file.write("Z-axis position: {0:.3f} in.\n".format(z))
    file.write("Length of corrosion: {0:.3f} in.\n".format(length))
    file.write("Width of corrosion: {0:.3f} in.\n".format(width))
    file.write("Depth: {0:.3f} in.\n\n".format(depth))
    file.close()

def main(args):
    #Try to run main from sys arguments
    try:
    #Inputs from system arguments
        filehandle = args[1]
        print("\nOpening and processing '{0}'...".format(filehandle))
        val = int(args[2])
        print("Threshold value is set at: {} (recommended around 80)".format(val))
        inverted = str(args[3])
        if not (inverted == 'black' or inverted == 'white'):
            print("Please enter 'black' or 'white'.")
            sys.exit(1)
        print("Inverse filtering: {}".format(inverted))

        num = input("Please enter anamoly number: ")
        o_clock = input("Please enter o'clock position: ")
        depth = float(input("Please enter depth (mm): "))/25.4
        type = input("Type of degredation (corrosion/coating damage): ")
        distance = float(input("Distance from corrosion site (cm): "))
        z_position = float(input("Z-axis position (cm): "))
        distance_sensor = input("Distance sensor (front/back): ")
        if distance_sensor != "front" and distance_sensor != "back":
            print("Error: Invalid sensor. Please select either front or back sensor.")
            sys.exit(1)
        image_sensor = input("Image sensor (front/back): ")
        if image_sensor != "front" and image_sensor != "back":
            print("Error: Invalid sensor. Please select either front or back sensor.")
            sys.exit(1)

        #Opening filehandle for reading and saving in memory
        image = cv.imread(filehandle)
        cv.imshow("Raw Image", image)
        cv.waitKey(0)
        cv.destroyWindow("Raw Image")

        filtered = gray_filter(image, val, inverted)
        #cb_blur = YCrCb_filter(image, channel='Cb', thresh=127)
        characteristics = connected_component(filtered, image)

        anomalies = []
        for index, anomaly in enumerate(characteristics):
            width_corrosion, height_corrosion, displacement = compute_length(image, anomaly[0], anomaly[1], anomaly[2], distance, z_position, distance_sensor, image_sensor)
            anomalies.append([width_corrosion, height_corrosion, displacement])

            print("\nSite {0}:".format(index))
            print("Anamoly {0}".format(num))
            print("{} site:".format(type))
            print("O'clock position: {0}".format(o_clock))
            print("Z-axis position: {0:.3f} in.".format(displacement))
            print("Length of corrosion: {0:.3f} in.".format(height_corrosion))
            print("Width of corrosion: {0:.3f} in.".format(width_corrosion))
            print("Depth: {0:.3f} in.".format(depth))
            filehandle = "{0}_output.txt".format(num)
            # first_write = 1
            #if os.path.exists(filehandle) and first_write == 1:
            #    os.remove(filehandle)
            #    write_output(filehandle, num, o_clock, displacement, height_corrosion, width_corrosion, depth)
            #    first_write = 0
            #else:
            #   write_output(filehandle, num, o_clock, displacement, height_corrosion, width_corrosion, depth)
            write_output(filehandle, num, o_clock, displacement, height_corrosion, width_corrosion, depth)

    except IndexError:
        print("\n6 arguments should be present")
        print("Usage: python3 length_computation.py [file] [thresh] [inv]")
        print("1. Filename to be processed")
        print("2. Threshold value for filtering")
        print("3. Inverse filtering (False by default)")
        sys.exit(3)



if __name__ == "__main__":
    main(sys.argv)
