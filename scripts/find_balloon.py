#
# find_balloon.py - find the pixel location of red balloons (although currently hard coded colour filters search for yellow circles)
#
#   Start from command line using 'python find_balloon.py'.
#   The program will run for 10 seconds and will create a video file, "find_balloon.avi", which contains the original video along with
#   super-imposed circles showing the where circles were detected
#
#   How to use:
#       run colour_finder.py to find the best min and max Hue, Saturation and Brightness (aka Value) levels.  Enter these into h_low, h_high, etc below
#       run this script and hold the object in front of the camera and move it around
#       check the find_balloon.avi file to ensure the super-imposed circles accurately follow the object

import sys
from time import time
import cv2
import cv2.cv
import numpy as np

# define image resolution
img_width = 640
img_height = 480

def get_camera():
    # setup video capture
    video_capture = cv2.VideoCapture(0)
    video_capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,img_width)
    video_capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,img_height)

    # check we can connect to camera
    if not video_capture.isOpened():
        print "failed to open camera, exiting!"
        sys.exit(0)

    return video_capture

def open_video_writer():
    # Define the codec and create VideoWriter object
    #ex = -1 # will display pop-up requesting user choose the encoder
    # MJPG seems to work on my linux box (though it is big)
    ex = int(cv2.cv.CV_FOURCC('M','J','P','G'))
    video_writer = cv2.VideoWriter('find_balloon.avi', ex, 25, (img_width,img_height))

    return video_writer

def main():
    video_capture = get_camera()
    video_writer = open_video_writer()

    # default colour filters (this is for a yellow tennis ball)
    h_low = 23
    h_high = 96
    s_low = 82
    s_high = 160
    v_low = 141
    v_high = 255

    # get start time
    start_time = time()

    # loop for 10 seconds looking for circles
    while(time() - start_time < 10):

        # Take each frame
        _, frame = video_capture.read()

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

        circles = cv2.HoughCircles(grey_res,cv2.cv.CV_HOUGH_GRADIENT,1,50,param1=50,param2=30,minRadius=0,maxRadius=0)
        #circles = cv2.HoughCircles(grey_res,cv2.cv.CV_HOUGH_GRADIENT,1,20)

        # check if any circles were found
        if not (circles is None):
            #print(circles)
            # draw circles around the circles
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

        # uncomment line below to see image with super-imposed circles in real-time
        #cv2.imshow('frame',frame)

        # write the flipped frame
        video_writer.write(frame)

# uncomment line below if window with real-time video was displayed
#cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
