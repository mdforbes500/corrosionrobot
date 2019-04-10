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
import sys

def main(args):
    #Inputs from system arguments
    filehandle = args[1]
    print("Opening and processing '{0}'...".format(filehandle))
    distance = float(args[2]) #cm
    print("Distance from object: {0} cm".format(distance))

    #Opening filehandle for reading and saving in memory as grayscale image
    # then applying gaussian blur to smooth image boundaries
    image = cv.imread(filehandle)
    image_gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    blurred = cv.GaussianBlur(image_gray, (11, 11), 0)
    cv.imshow("Gaussian Filtering", blurred)
    cv.waitKey(0)
    cv.destroyWindow("Gaussian Filtering")

    #===============================CORROSION=================================#
    #Softening threshold values to again lower noisy edges
    ret,corrosion_thresh = cv.threshold(blurred,40,255,0)
    corrosion_thresh = cv.erode(corrosion_thresh, None, iterations=2)
    corrosion_thresh = cv.dilate(corrosion_thresh, None, iterations=4)
    corrosion_thresh = cv.bitwise_not(corrosion_thresh) #reverse colors

    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    corrosion_labels = measure.label(corrosion_thresh, neighbors=8, background=0)
    corrosion_mask = np.zeros(corrosion_thresh.shape, dtype="uint8")

    # loop over the unique components
    for corrosion_label in np.unique(corrosion_labels):
        #if this is the background label, ignore it
        if corrosion_label == 0:
            continue

	# otherwise, construct the label mask and count the
	# number of pixels
    corrosion_label_mask = np.zeros(corrosion_thresh.shape, dtype="uint8")
    corrosion_label_mask[corrosion_labels == corrosion_label] = 255
    num_pixels = cv.countNonZero(corrosion_label_mask)

    # if the number of pixels in the component is sufficiently
    # large, then add it to our mask of "large blobs"
    if num_pixels > 300:
        corrosion_mask = cv.add(corrosion_mask, corrosion_label_mask)

    cv.imshow('Corrosion Threshold image', corrosion_mask)
    cv.waitKey(0)
    cv.destroyWindow('Corrosion Threshold image')
    if (cv.__version__ == "3.3.0"):
        (_,corrosion_contours,heirarchy_c)= cv.findContours(corrosion_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    else:
        (corrosion_contours,heirarchy_c)= cv.findContours(corrosion_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnt_c = corrosion_contours[0]
    x_c,y_c,w_c,h_c = cv.boundingRect(cnt_c)
    print("Corrosion width is: {0}\nCorrosion height is: {1}".format(w_c, h_c))

    #Plot bounding box
    rect_c = cv.minAreaRect(cnt_c)
    box_c = cv.boxPoints(rect_c)
    box_c = np.int0(box_c)
    im_corrosion = cv.drawContours(image,[box_c],0,(0,0,255),2)
    cv.imshow('Corrosion Contours', im_corrosion)
    cv.waitKey(0)
    cv.destroyWindow('Corrosion Contours')

    #===============================COATING===================================#
    ret2,coating_thresh = cv.threshold(blurred,127,255,0)
    coating_thresh = cv.erode(coating_thresh, None, iterations=2)
    coating_thresh = cv.dilate(coating_thresh, None, iterations=2)
    coating_thresh = cv.bitwise_not(coating_thresh) #reverse colors
    #coating_thresh = cv.bitwise_not(coating_thresh)

    # perform a connected component analysis on the thresholded
    # image, then initialize a mask to store only the "large"
    # components
    coating_labels = measure.label(coating_thresh, neighbors=8, background=0)
    coating_mask = np.zeros(coating_thresh.shape, dtype="uint8")

    # loop over the unique components
    for coating_label in np.unique(coating_labels):
        #if this is the background label, ignore it
        if coating_label == 0:
            continue

	# otherwise, construct the label mask and count the
	# number of pixels
    coating_label_mask = np.zeros(coating_thresh.shape, dtype="uint8")
    coating_label_mask[coating_labels == coating_label] = 255
    num_pixels = cv.countNonZero(coating_label_mask)

    # if the number of pixels in the component is sufficiently
    # large, then add it to our mask of "large blobs"
    if num_pixels > 300:
        coating_mask = cv.add(coating_mask, coating_label_mask)

    cv.imshow('Coating Threshold image', coating_mask)
    cv.waitKey(0)
    cv.destroyWindow('Coating Threshold image')

    if (cv.__version__ == "3.3.0"):
        (_,coating_contours, heirarchy_ct) = cv.findContours(coating_mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    else:
        (coating_contours, heirarchy_ct) = cv.findContours(coating_mask, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    cnt_ct = coating_contours[0]
    x_ct,y_ct,w_ct,h_ct = cv.boundingRect(cnt_ct)
    print("Coating width is: {0}\nCoating height is: {1}".format(w_ct, h_ct))

    #Plot bounding box
    rect_ct = cv.minAreaRect(cnt_ct)
    box_ct = cv.boxPoints(rect_ct)
    box_ct = np.int0(box_ct)
    im_coating = cv.drawContours(image,[box_ct],0,(0,0,255),2)
    cv.imshow('Coating Contours', im_coating)
    cv.waitKey(0)
    cv.destroyWindow('Coating Contours')
    #===========================Length Computation============================#
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

    #Computation of width and height of corrosion anamoly
    corrosion_width = px_width*w_c/2.54
    corrosion_height = px_height*h_c/2.54
    print("Corrosion width is: {0.3}\nCorrosion height is: {1.3}".format(corrosion_width, corrosion_height))

    #Computation of width and height of coating damage
    coating_width = px_width*w_ct/2.54
    coating_height = px_height*h_ct/2.54
    print("Coating width is: {0.3}\nCoating height is: {1.3}".format(coating_width, coating_height))

if __name__ == "__main__":
    main(sys.argv)
