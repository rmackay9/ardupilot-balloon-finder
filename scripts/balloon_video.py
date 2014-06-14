"""
balloon_video.py

This file includes functions to:
    Initialise the camera
    Initialise the output video

Image size is held in the balloon_finder.cnf
"""

import sys
import math
import cv2
import balloon_config

class BalloonVideo:

    def __init__(self):
        # get image resolution
        self.img_width = balloon_config.config.get_integer('camera','width',640)
        self.img_height = balloon_config.config.get_integer('camera','height',480)

        # get image center
        self.img_center_x = self.img_width / 2
        self.img_center_y = self.img_height / 2
        
        # define field of view
        self.cam_hfov = balloon_config.config.get_float('camera','horizontal-fov',70.42)
        self.cam_vfov = balloon_config.config.get_float('camera','vertical-fov',43.3)

        # define video output filename
        self.video_filename = balloon_config.config.get_string('camera','video_output_file','find_balloon.avi')

    # __str__ - print position vector as string
    def __str__(self):
        return "BalloonVideo Object W:%d H:%d" % (self.img_width, self.img_height)

    # get_camera - initialises camera and returns VideoCapture object 
    def get_camera(self):
        # setup video capture
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.img_width)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.img_height)

        # check we can connect to camera
        if not self.camera.isOpened():
            print "failed to open camera, exiting!"
            sys.exit(0)

        return self.camera

    # open_video_writer - begin writing to video file
    def open_video_writer(self):
        # Define the codec and create VideoWriter object
        # Note: setting ex to -1 will display pop-up requesting user choose the encoder
        ex = int(cv2.cv.CV_FOURCC('M','J','P','G'))
        self.video_writer = cv2.VideoWriter(self.video_filename, ex, 25, (self.img_width,self.img_height))
    
        return self.video_writer

    # pixels_to_angle_x - converts a number of pixels into an angle in radians 
    def pixels_to_angle_x(self, num_pixels):
        return num_pixels * math.radians(self.cam_hfov) / self.img_width
    
    # pixels_to_angle_y - converts a number of pixels into an angle in radians 
    def pixels_to_angle_y(self, num_pixels):
        return num_pixels * math.radians(self.cam_vfov) / self.img_height
    
    # angle_to_pixels_x - converts a horizontal angle (i.e. yaw) to a number of pixels
    #    angle : angle in radians
    def angle_to_pixels_x(self, angle):
        return int(angle * self.img_width / math.radians(self.cam_hfov))
    
    # angle_to_pixels_y - converts a vertical angle (i.e. pitch) to a number of pixels
    #    angle : angle in radians 
    def angle_to_pixels_y(self, angle):
        return int(angle * self.img_height / math.radians(self.cam_vfov))

    # main - tests BalloonVideo class
    def main(self):
        print "got here!"
        camera = self.get_camera()
        print "a2p 10 = %f" % self.angle_to_pixels_x(10)
        print "p2a 10 = %f" % self.pixels_to_angle_x(10)

# create a single global object
balloon_video = BalloonVideo()

if __name__ == "__main__":
    balloon_video.main()