#! /usr/bin/env python
import rospy
import FindArea.srv

def find_area():
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
    image = cv.imread(filehandle)
    image_gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
#    print(image_gray)
    cv.imshow('Gray image', image_gray)
    cv.waitKey(0)

    # Determining the CMOS width and height in pixels
    width = np.size(image_gray, 1) #px
    print("Width of image is : {} pixels".format(width))
    height = np.size(image_gray, 0) #px
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

    sides = math.sqrt(corrosion_area)
    adj = math.sqrt(distance**2 - sides**2/4)
    theta = 2*math.atan(sides/(2*adj))
    arclength = distance*theta
    adjusted_area = arclength*image_width

    print("Number of pixels: {0:d} pixels".format(counter))
    print("Number of corroded pixels: {0:d} pixels".format(CountPixelB))
    print("Total Area of Corrosion: {0:.{1}f} cm^2".format(corrosion_area,3))
#    print("Adjusted area with curvature: {0:.{1}f} cm^2".format(adjusted_area,3))

if __name__ == "__main__":
    find_area()
