#
# colour_finder.py - helps find the min and max Hue, Saturation and Brightness (aka Value) of a desired object
#
#   Start from command line using 'python colour_finder.py', select a window and press ESC at any time to quit
#
#   4 windows will be displayed:
#       Colour Filters : display high and low colour filters for Hue, Saturation and Brightness (aka Value)
#       Original : displays the raw image from the camera
#       Mask : displays black and white mask where black parts will be removed, white parts will remain
#       Filtered Result: Original image with Mask applied.  Only desired object should be visible
#
#   How to use:
#       Start the program and hold the object in front of the camera (the object should not fill the entire screen)
#       Increase the min and decrease the Max Hue, Saturation and Brightness trackbars so that only the desired object is shown in the Filtered Result
#       Record the values so they can be input manually into the usb_cam_test.py script

import cv2
import cv2.cv
import numpy as np

# define image resolution
img_width = 640
img_height = 480

# setup video capture
video_capture = cv2.VideoCapture(0)
video_capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,img_width)
video_capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,img_height)

# check we can connect to camera
if not video_capture.isOpened():
    print "failed to open camera, exiting!"
    sys.exit(0)

# call back for trackbar movements
def empty_callback(x):
    pass

# default filters -- all visible
h_low = 0
h_high = 255
s_low = 0
s_high = 255
v_low = 0
v_high = 255

# default filter -- yellow tennis ball
#h_low = 23
#h_high = 96
#s_low = 82
#s_high = 160
#v_low = 141
#v_high = 255
    
# create trackbars for color change
cv2.namedWindow('Colour Filters')
cv2.createTrackbar('Hue min','Colour Filters',h_low,255,empty_callback)
cv2.createTrackbar('Hue max','Colour Filters',h_high,255,empty_callback)
cv2.createTrackbar('Sat min','Colour Filters',s_low,255,empty_callback)
cv2.createTrackbar('Sat max','Colour Filters',s_high,255,empty_callback)
cv2.createTrackbar('Bgt min','Colour Filters',v_low,255,empty_callback)
cv2.createTrackbar('Bgt max','Colour Filters',v_high,255,empty_callback)

while(1):

    # Take each frame
    _, frame = video_capture.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # get latest trackbar positions
    h_low = cv2.getTrackbarPos('Hue min','Colour Filters')
    h_high = cv2.getTrackbarPos('Hue max','Colour Filters')
    s_low = cv2.getTrackbarPos('Sat min','Colour Filters')
    s_high = cv2.getTrackbarPos('Sat max','Colour Filters')
    v_low = cv2.getTrackbarPos('Bgt min','Colour Filters')
    v_high = cv2.getTrackbarPos('Bgt max','Colour Filters')

    # use trackbar positions to filter image
    colour_low = np.array([h_low,s_low,v_low])
    colour_high = np.array([h_high,s_high,v_high])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, colour_low, colour_high)

    # blur the result
    #mask = cv2.medianBlur(mask,9)

    # Erode
    erode_kernel = np.ones((3,3),np.uint8);
    eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)

    # dilate
    dilate_kernel = np.ones((10,10),np.uint8);
    dilate_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= dilate_img)
            
    cv2.imshow('Original',frame)
    cv2.imshow('Mask',mask)
    cv2.imshow('Filtered Result',res)
    
    #cv2.imshow('grey_res',grey_res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()