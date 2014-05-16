#
#   raw.py - helps find the min and max Hue, Saturation and Brightness (aka Value) of a desired object
#
#   Start from command line using 'raw.py', select a window and press ESC at any time to quit
#
#   1 window will be displayed at startup:
#       Colour Filters : display high and low colour filters for Hue, Saturation and Brightness (aka Value) also displays the tracking results.
#       the first 6 sliders are disabled until you set the  7th slider to OFF
# 
#   3 Windows will be displayed when the 7th slider is set to OFF
#       Colour Filters : displays the raw image from the camera
#       Mask : displays black and white mask where black parts will be removed, white parts will remain
#       Filtered Result: Original image with Mask applied.  Only desired object should be visible
#
#   How to use:
#       Start the program and hold the object in front of the camera (the object should not fill the entire screen)
#       Increase the min and decrease the Max Hue, Saturation and Brightness trackbars so that only the desired object is shown in the Filtered Result
#       set the 7th slider to ON, to see it tracking.
#       the 8th slider selects if only the largest circle is chosen.

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
switch = 1
num_circle = 1
# default filter -- yellow tennis ball (Randy's default)
h_low = 0
h_high = 22
s_low = 168
s_high = 255
v_low = 147
v_high = 255
#default filter -- orange paint lid (Phils Default)
h_low = 0
h_high = 22
s_low = 168
s_high = 255
v_low = 147
v_high = 255
    
# create trackbars for color change
cv2.namedWindow('Colour Filters')
cv2.createTrackbar('Hue min','Colour Filters',h_low,255,empty_callback)
cv2.createTrackbar('Hue max','Colour Filters',h_high,255,empty_callback)
cv2.createTrackbar('Sat min','Colour Filters',s_low,255,empty_callback)
cv2.createTrackbar('Sat max','Colour Filters',s_high,255,empty_callback)
cv2.createTrackbar('Bgt min','Colour Filters',v_low,255,empty_callback)
cv2.createTrackbar('Bgt max','Colour Filters',v_high,255,empty_callback)
cv2.createTrackbar('0 : OFF \n1 : ON','Colour Filters',switch,1,empty_callback)
cv2.createTrackbar('1 Circle \nAll Circles','Colour Filters',num_circle,1,empty_callback)

#create basic output window
out_win = np.zeros((300,512,3),np.uint8)


while(1):

    # Take each frame
    _, frame = video_capture.read()

    switch = cv2.getTrackbarPos('0 : OFF \n1 : ON','Colour Filters')
    num_circle = cv2.getTrackbarPos('1 Circle \nAll Circles','Colour Filters')

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
    
    # create a grey version of the result
    grey_res = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

    # threshold it to a black and white image
    #thresh_used, grey_res = cv2.threshold(grey_res,10,255,cv2.THRESH_BINARY)

    #grey_res = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    # blur it to reduce false circles
    grey_res = cv2.medianBlur(grey_res,5)

    if (switch == 1):
        circles = cv2.HoughCircles(grey_res,cv2.cv.CV_HOUGH_GRADIENT,1,50,param1=50,param2=30,minRadius=0,maxRadius=0)
    	#circles = cv2.HoughCircles(grey_res,cv2.cv.CV_HOUGH_GRADIENT,1,20)

        # check if any circles were found
    	if not (circles is None):
        	# print(circles)
        	# draw circles around the circles
        	circles = np.uint16(np.around(circles))
                biggest = 0
                big_circle = 0
                for i in circles[0,:]:
                    if (i[2] > biggest):
                        biggest = i[2]
                        # print biggest 
                        big_circle = i
                if (num_circle == 1):
                   cv2.line(frame,(big_circle[0],1),(big_circle[0],640),5000,2)
                   cv2.line(frame,(1,big_circle[1]),(640,big_circle[1]),5000,2)
                else:
                    for i in circles[0,:]:
            		# draw the outer circle
            		cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),-1)
            		cv2.line(frame,(i[0],1),(i[0],640),5000,2)            
	    		cv2.line(frame,(1,i[1]),(640,i[1]),5000,2)
	   		# draw the center of the circle
           		# cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
        # print switch        
    else:
        # get latest trackbar positions
        h_low = cv2.getTrackbarPos('Hue min','Colour Filters')
        h_high = cv2.getTrackbarPos('Hue max','Colour Filters')
        s_low = cv2.getTrackbarPos('Sat min','Colour Filters')
        s_high = cv2.getTrackbarPos('Sat max','Colour Filters')
        v_low = cv2.getTrackbarPos('Bgt min','Colour Filters')
        v_high = cv2.getTrackbarPos('Bgt max','Colour Filters')
        switch = cv2.getTrackbarPos('0 : OFF \n1 : ON','Colour Filters')
        num_circle = cv2.getTrackbarPos('1 Circle \nAll Circles','Colour Filters')
        
        cv2.imshow('Mask',mask)
        cv2.imshow('Filtered Result',res)
        #cv2.imshow('grey_res',grey_res)
   
    cv2.imshow('Colour Filters',frame)

    # set escape key for exit
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
