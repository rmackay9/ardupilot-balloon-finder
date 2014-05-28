#
# colour_finder_web.py - helps find the min and max Hue, Saturation and Brightness (aka Value) of a desired object
#
#   Start from command line using 'python colour_finder_web.py'
#
#   Access the final image at http://<ipaddress>:8081
#
#   Adjust the colour filters directly in the balloon_config.cnf file
#       h-low : hue min
#       h-high: hue max
#       s-low : saturation min
#       s-high: saturation max
#       v-low : value (aka brightness) min
#       v-high: value (aka brightness) max
#

import time
import cv2
import numpy as np
import balloon_config
from web_server import Webserver
from balloon_video import balloon_video
from find_balloon import BalloonFinder

class ColourFinder:

    # constructor
    def __init__(self):
        # check if config exists and if not create it
        if (balloon_config.config.get_integer('balloon','h-low',-1) == -1):
            balloon_config.config.set_integer('balloon','h-low',BalloonFinder.default_h_low)
            balloon_config.config.set_integer('balloon','h-high',BalloonFinder.default_h_high)
            balloon_config.config.set_integer('balloon','s-low',BalloonFinder.default_s_low)
            balloon_config.config.set_integer('balloon','s-high',BalloonFinder.default_s_high)
            balloon_config.config.set_integer('balloon','v-low',BalloonFinder.default_v_low)
            balloon_config.config.set_integer('balloon','v-high',BalloonFinder.default_v_high)
            balloon_config.config.save();
            print "Initialised colour filters to default in balloon_finder.cnf file"

        # initialise config last read time
        self.config_last_read = time.time()

        # read config file
        self.read_config_file(True)

        # frame that will be displayed by webserver 
        self.frame_filtered = None

    # read_config_file - reads latest colour filters from config file
    #     force : boolean.  if set to True will always read from config file even if a read has been performed recently
    def read_config_file(self, force_read):

        # check at least one second has passed since the last read
        if (force_read or (time.time() - self.config_last_read > 1)):
            # re-read the config file
            balloon_config.config.read()

            # load colour filters from config file
            self.h_low = balloon_config.config.get_integer('balloon','h-low',BalloonFinder.default_h_low)
            self.h_high = balloon_config.config.get_integer('balloon','h-high',BalloonFinder.default_h_high)
            self.s_low = balloon_config.config.get_integer('balloon','s-low',BalloonFinder.default_s_low)
            self.s_high = balloon_config.config.get_integer('balloon','s-high',BalloonFinder.default_s_high)
            self.v_low = balloon_config.config.get_integer('balloon','v-low',BalloonFinder.default_v_low)
            self.v_high = balloon_config.config.get_integer('balloon','v-high',BalloonFinder.default_v_high)

            # store time config file was last read (we will check it once per second)
            self.config_last_read = time.time()

    # run - main routine to help user find colour filters
    def run(self):

        try:
            print "starting web server"
            # initialise web server which will display final image
            web = Webserver(balloon_config.config.parser, (lambda : self.frame_filtered))
    
            print "initialising camera"
            # initialise video capture
            camera = balloon_video.get_camera()
    
            print "Ready to go!"

            while(True):
                # get a frame
                _, frame = camera.read()
    
                # Convert BGR to HSV
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
                # get latest colour filters
                self.read_config_file(False)
    
                # use trackbar positions to filter image
                colour_low = np.array([self.h_low,self.s_low,self.v_low])
                colour_high = np.array([self.h_high,self.s_high,self.v_high])
            
                # Threshold the HSV image
                mask = cv2.inRange(hsv_frame, colour_low, colour_high)
            
                # Erode
                erode_kernel = np.ones((3,3),np.uint8);
                eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)
            
                # dilate
                dilate_kernel = np.ones((10,10),np.uint8);
                dilated_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)
            
                # Bitwise-AND mask and original image
                self.frame_filtered = cv2.bitwise_and(frame,frame, mask=dilated_img)

        # handle interrupts
        except:
            print "interrupted, exiting"

        # exit and close web server
        print "exiting..."
        web.close()

# create global colour_finder object and run it
colour_finder = ColourFinder()
colour_finder.run()