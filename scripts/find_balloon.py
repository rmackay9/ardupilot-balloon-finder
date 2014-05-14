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
import math

# define image resolution
img_width = 640
img_height = 480

# calculate center of image in pixels
img_center_x = img_width / 2
img_center_y = img_height / 2    

# define field of view
cam_hfov = 70.42
cam_vfov = 43.3

# define expected balloon radius in meters
balloon_radius_expected = 0.5

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

def analyse_frame(frame):
    # default colour filters (this is for the red sparkfun balloon)
    h_low = 154
    h_high = 195
    s_low = 75
    s_high = 255
    v_low = 63
    v_high = 191

    balloon_found = False
    balloon_x = 0
    balloon_y = 0
    balloon_radius = 0

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # use trackbar positions to filter image
    colour_low = np.array([h_low,s_low,v_low])
    colour_high = np.array([h_high,s_high,v_high])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, colour_low, colour_high)

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

    # blur it to reduce false circles
    grey_res = cv2.medianBlur(grey_res,5)

    circles = cv2.HoughCircles(grey_res,cv2.cv.CV_HOUGH_GRADIENT,1,50,param1=50,param2=30,minRadius=0,maxRadius=0)
    #circles = cv2.HoughCircles(grey_res,cv2.cv.CV_HOUGH_GRADIENT,1,20)

    # check if any circles were found
    if not (circles is None):

        # flag we have found at least one circle
        balloon_found = True

        # reset size and number of largest circle
        biggest_radius = 0
        biggest_circle_num = 0

        # find largest circle        
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            if (i[2] > biggest_radius):
                biggest_circle = i
                biggest_radius = i[2]

        # draw circle around the largest circle
        cv2.circle(frame,(biggest_circle[0],biggest_circle[1]),biggest_circle[2],(0,255,0),2)
       # draw the center of the circle
        cv2.circle(frame,(biggest_circle[0],biggest_circle[1]),2,(0,0,255),3)

        balloon_x = biggest_circle[0]
        balloon_y = biggest_circle[1]
        balloon_radius = biggest_circle[2]

    # return results
    return balloon_found, balloon_x, balloon_y, balloon_radius

# image_pos_to_angle - converts an x, y position into an angle
# returns a horizontal and vertical angle from the center
# i.e. 0,0 is directly ahead of the camera
# +ve x = right of center, -ve x = left of center
# +ve y = above center, -ve y = below center
def image_pos_to_angle(xpos, ypos):
    x_angle = (float(xpos) / float(img_width) * cam_hfov) - (cam_hfov/2)
    y_angle = (cam_vfov/2) - (float(ypos) / float(img_height) * cam_vfov)
    return x_angle, y_angle

# rotate_pos - rotates a point by the provided angle around the center of the image
# xpos should be in the range of -cam_hfov ~ +cam_hfov
# ypos should be in the rnage of -cam_vfov ~ +cam_vfov
# returns a modified x,y position
# +ve angle = rotate clockwise
# -ve angle = rotate counter clockwise
def rotate_pos(xpos, ypos, angle_in_radians):
    cos_ang = math.cos(angle_in_radians)
    sin_ang = math.sin(angle_in_radians)
    x_rotated = xpos * cos_ang - ypos * sin_ang
    y_rotated = xpos * sin_ang + ypos * cos_ang
    return x_rotated, y_rotated    

# add_artificial_horizon - adds artificial horizon to image
def add_artificial_horizon(frame, roll_in_radians, pitch_in_radians):
    ah1_x = -200
    ah1_y = 0
    ah2_x = 200
    ah2_y = 0
    # rotate by -ve roll angle
    ah1_x_rot, ah1_y_rot = rotate_pos(ah1_x, ah1_y, -roll_in_radians)
    ah2_x_rot, ah2_y_rot = rotate_pos(ah2_x, ah2_y, -roll_in_radians)
    # shift down by by -ve pitch angle
    pitch_pixel_shift = int(math.degrees(pitch_in_radians) / float(cam_vfov) * img_height)
    cv2.line(frame,(int(ah1_x_rot)+img_center_x,int(ah1_y_rot)+img_center_y+pitch_pixel_shift),(int(ah2_x_rot)+img_center_x,int(ah2_y_rot)+img_center_y+pitch_pixel_shift),5000,2)

# pos_to_direction - converts a pixel location to a direction vector
#    returns vertical angle (-ve = down, +ve = upwards) and absolute heading (-180 ~ 180 degrees) towards target
#    xpos should be a number from 0 ~ img_width
#    ypos should be a number from 0 ~ img_height
def pos_to_direction(xpos, ypos, roll_in_radians, pitch_in_radians, yaw_in_radians):
    # rotate position by +ve roll angle
    x_rotated, y_rotated = rotate_pos(xpos - img_width/2, ypos - img_height/2, roll_in_radians)
    # calculate vertical pixel shift from pitch angle
    pitch_pixel_shift = int(math.degrees(pitch_in_radians) / float(cam_vfov) * img_height)
    pitch_dir = (-y_rotated + pitch_pixel_shift) / float(img_height) * cam_vfov
    # calculate yaw shift in degrees
    yaw_dir = x_rotated / float(img_width) * float(cam_hfov) + math.degrees(yaw_in_radians)
    # return vertical angle to target and heading
    return pitch_dir, yaw_dir

# get_distance_from_pixels - returns distance to balloon in meters given number of pixels in image and expected 0.5m radius
def get_distance_from_pixels(radius_in_pixels):
    # avoid divide by zero by returning 9999.9 meters for zero sized object 
    if (radius_in_pixels == 0):
        return 9999.9
    # convert num_pixels to angular size
    return balloon_radius_expected / pixel_x_to_angle(radius_in_pixels)

# angle_to_pixel_x - converts a horizontal angle (i.e. yaw) to a number of pixels 
def angle_to_pixel_x(angle_in_radians):
    return int(angle_in_radians * img_width / math.radians(cam_hfov))

# angle_to_pixel_x - converts a horizontal angle (i.e. yaw) to a number of pixels 
def pixel_x_to_angle(num_pixels):
    return num_pixels * math.radians(cam_hfov) / img_width

# project_position - calculates the position in m given a yaw angle, pitch angle and distance
def project_position(pitch_in_radians, yaw_in_radians, distance_m):
    cos_pitch = math.cos(pitch_in_radians)
    x = distance_m * math.cos(yaw_in_radians) * cos_pitch
    y = distance_m * math.sin(yaw_in_radians) * cos_pitch
    z = distance_m * math.sin(pitch_in_radians)
    return (x,y,z)

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
