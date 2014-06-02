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
import time
import cv2
import numpy
import math
import balloon_config
from web_server import Webserver
from balloon_video import balloon_video
import balloon_utils
from position_vector import PositionVector

class BalloonFinder(object):

    # default colours for red balloon
    default_h_low = 154
    default_h_high = 195
    default_s_low = 75
    default_s_high = 255
    default_v_low = 63
    default_v_high = 191
    
    def __init__(self):

        # define expected balloon radius in meters
        self.balloon_radius_expected = balloon_config.config.get_float('balloon','radius_cm',0.5)

        # colour filters for balloon
        self.filter_low = numpy.array([balloon_config.config.get_integer('balloon','h-low',BalloonFinder.default_h_low),
                                       balloon_config.config.get_integer('balloon','s-low',BalloonFinder.default_s_low),
                                       balloon_config.config.get_integer('balloon','v-low',BalloonFinder.default_v_low)])

        self.filter_high = numpy.array([balloon_config.config.get_integer('balloon','h-high',BalloonFinder.default_h_high),
                                        balloon_config.config.get_integer('balloon','s-high',BalloonFinder.default_s_high),
                                        balloon_config.config.get_integer('balloon','v-high',BalloonFinder.default_v_high)])

        self.frame = None

    # analyse_frame - look for balloon in image using SimpleBlobDetector
    #    returns:
    #        found: boolean which is true if balloon if found
    #        x: an integer holding the horizontal position in pixels of the center of the balloon on image  
    #        y: an integer holding the vertical position in pixels of the center of the balloon on image
    #        radius: a float(?) holding the radius of the balloon in pixels
    def analyse_frame(self,frame):
        balloon_found = False
        balloon_x = 0
        balloon_y = 0
        balloon_radius = 0
    
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # Threshold the HSV image
        mask = cv2.inRange(hsv, self.filter_low, self.filter_high)
    
        # Erode
        erode_kernel = numpy.ones((3,3),numpy.uint8);
        eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)
    
        # dilate
        dilate_kernel = numpy.ones((10,10),numpy.uint8);
        dilate_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)
    
        # blog detector
        blob_params = cv2.SimpleBlobDetector_Params()
        blob_params.minDistBetweenBlobs = 50
        blob_params.filterByInertia = False
        blob_params.filterByConvexity = False
        blob_params.filterByColor = True
        blob_params.blobColor = 255
        blob_params.filterByCircularity = False
        blob_params.filterByArea = False
        #blob_params.minArea = 20
        #blob_params.maxArea = 500
        blob_detector = cv2.SimpleBlobDetector(blob_params)
        keypts = blob_detector.detect(dilate_img)
    
        # draw centers of all keypoints in new image
        #blob_img = cv2.drawKeypoints(frame, keypts, color=(0,255,0), flags=0)
    
        # find largest blob
        if len(keypts) > 0:
            kp_max = keypts[0]
            for kp in keypts:
                if kp.size > kp_max.size:
                    kp_max = kp
    
            # draw circle around the largest blob
            cv2.circle(frame,(int(kp_max.pt[0]),int(kp_max.pt[1])),int(kp_max.size),(0,255,0),2)
    
            # set the balloon location
            balloon_found = True
            balloon_x = kp_max.pt[0]
            balloon_y = kp_max.pt[1]
            balloon_radius = kp_max.size
    
        # return results
        return balloon_found, balloon_x, balloon_y, balloon_radius

    # add_artificial_horizon - adds artificial horizon to an image using the vehicle's attitude
    def add_artificial_horizon(self, frame, vehicle_roll, vehicle_pitch):
        # horizon line is 200 pixels in either direction from center of image
        ah1_x, ah1_y = balloon_utils.rotate_xy(balloon_video.img_center_x - 200, balloon_video.img_center_y, -vehicle_roll)
        ah2_x, ah2_y = balloon_utils.rotate_xy(balloon_video.img_center_x + 200, balloon_video.img_center_y, -vehicle_roll)
        # shift down by by -ve pitch angle
        pitch_pixel_shift = int(math.degrees(vehicle_pitch) / float(balloon_video.cam_vfov) * balloon_video.img_height)
        # draw line
        cv2.line(frame,(int(ah1_x),int(ah1_y)+pitch_pixel_shift),(int(ah2_x),int(ah2_y)+pitch_pixel_shift),5000,2)

    #
    # Below are functions that together can be used to convert the location on the image
    # to an estimate of the balloon's actual earth-frame location
    #

    # pixels_to_direction - converts a pixel location and vehicle attitude to an earth-frame pitch angle and heading
    #    pixels_x should be a number from 0 ~ img_width
    #    pixels_y should be a number from 0 ~ img_height
    #    roll_in_radians, pitch_in_radians, yaw_in_radiasn should be the vehicle's roll, pitch and yaw angles in radians
    #    returns vector towards target:
    #        vertical angle (-ve = down, +ve = upwards) in degrees
    #        absolute heading (in degrees))
    def pixels_to_direction(self, pixels_x, pixels_y, vehicle_roll, vehicle_pitch, vehicle_yaw):
        # rotate position by +ve roll angle
        x_rotated, y_rotated = balloon_utils.rotate_xy(pixels_x - balloon_video.img_width/2, pixels_y - balloon_video.img_height/2, vehicle_roll)
        # calculate vertical pixel shift from pitch angle
        pitch_pixel_shift = int(math.degrees(vehicle_pitch) / float(balloon_video.cam_vfov) * balloon_video.img_height)
        pitch_dir = (-y_rotated + pitch_pixel_shift) / float(balloon_video.img_height) * balloon_video.cam_vfov
        # calculate yaw shift in degrees
        yaw_dir = x_rotated / float(balloon_video.img_width) * float(balloon_video.cam_hfov) + math.degrees(vehicle_yaw)
        # return vertical angle to target and heading
        return pitch_dir, yaw_dir

    # project_position - calculates the position in m given a yaw angle, pitch angle and distance
    #    origin : PositionVector of the origin
    #    pitch : earth frame pitch angle from vehicle to object.  +ve = above vehicle, -ve = below vehicle
    #    yaw : earth frame heading.  +ve = clockwise from north, -ve = counter clockwise from north
    #    distance : distance from vehicle to object in meters
    def project_position(self, origin, pitch, yaw, distance):
        cos_pitch = math.cos(pitch)
        dx = distance * math.cos(yaw) * cos_pitch
        dy = distance * math.sin(yaw) * cos_pitch
        dz = distance * math.sin(pitch)
        ret = PositionVector(origin.x + dx,origin.y + dy, origin.z + dz)
        return ret

    # get_ef_velocity_vector - calculates the earth frame velocity vector in m/s given a yaw angle, pitch angle and speed in m/s
    #    pitch : earth frame pitch angle from vehicle to object.  +ve = above vehicle, -ve = below vehicle
    #    yaw : earth frame heading.  +ve = clockwise from north, -ve = counter clockwise from north
    #    velocity : scalar velocity in m/s
    def get_ef_velocity_vector(self, pitch, yaw, speed):
        cos_pitch = math.cos(pitch)
        x = speed * math.cos(yaw) * cos_pitch
        y = speed * math.sin(yaw) * cos_pitch
        z = speed * math.sin(pitch)
        return x,y,z

    # main - tests the BalloonFinder class
    def main(self):
        web = Webserver(balloon_config.config.parser, (lambda : self.frame))

        camera = balloon_video.get_camera()
        video_writer = balloon_video.open_video_writer()

        # get start time
        start_time = time.time()

        # loop for 10 seconds looking for circles
        while(time.time() - start_time < 20):

            # Take each frame
            _, frame = camera.read()
            self.frame = frame

            # is there the x & y position in frame of the largest balloon
            found_in_image, xpos, ypos, size = self.analyse_frame(frame)

            # display image
            cv2.imshow('frame',frame)

            # write the frame
            video_writer.write(frame)

            # exit if user presses ESC
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

        print "exiting..."
        web.close()

        # uncomment line below if window with real-time video was displayed
        cv2.destroyAllWindows()

        # release camera
        camera.release()

# create the global balloon_finder object
balloon_finder = BalloonFinder()

# run a test if this file is being invoked directly from the command line
if __name__ == "__main__":
    balloon_finder.main()
