#
# fake_balloon.py - creates an imagine of a balloon given a vehicle position, vehicle attitude and balloon position 
#
#   Start from command line using 'python balloon_finder.py'.
#   The program will run for 10 seconds and will create a video file, "balloon_finder.avi", which contains the original video along with
#   super-imposed circles showing the where circles were detected
#
#   How to use:
#       run colour_finder.py to find the best min and max Hue, Saturation and Brightness (aka Value) levels.  Enter these into h_low, h_high, etc below
#       run this script and hold the object in front of the camera and move it around
#       check the balloon_finder.avi file to ensure the super-imposed circles accurately follow the object

import sys
from time import time
import math
import cv2
import numpy
from dronekit import LocationGlobal
import balloon_config
from balloon_video import balloon_video
import balloon_utils
from position_vector import PositionVector
from find_balloon import balloon_finder

class BalloonSimulator(object):

    # constructor
    def __init__(self):

        # read fake balloon location from config file
        self.fake_balloon_location = LocationGlobal(balloon_config.config.get_float('fake-balloon', 'lat',-35.363274),
                                              balloon_config.config.get_float('fake-balloon', 'lon',149.164630),
                                              balloon_config.config.get_float('fake-balloon', 'alt',15))

        # fake balloon's colour is mid way between colour filter's low and high values
        h = (balloon_finder.filter_low[0] + balloon_finder.filter_high[0]) / 2
        s = (balloon_finder.filter_low[1] + balloon_finder.filter_high[1]) / 2
        v = (balloon_finder.filter_low[2] + balloon_finder.filter_high[2]) / 2

        # convert colour to BGR palette
        fake_balloon_colour_bgr = cv2.cvtColor(numpy.uint8([[[h,s,v]]]),cv2.COLOR_HSV2BGR)
        self.fake_balloon_colour_bgr_scalar = [fake_balloon_colour_bgr.item(0), fake_balloon_colour_bgr.item(1), fake_balloon_colour_bgr.item(2)]

        # fake balloon is same radius as actual balloon
        self.fake_balloon_radius = balloon_finder.balloon_radius_expected

        # background sky and ground colours
        self.background_sky_colour_bgr = (232, 228, 227)
        self.background_ground_colour_bgr_scalar = [87, 145, 158]

        # last iterations balloon radius
        self.last_balloon_radius = 0


    # get_background - returns a background image given a roll and pitch angle
    #     vehicle_roll and pitch are in radians
    def get_background(self, vehicle_roll, vehicle_pitch):

        # create sky coloured image
        image = numpy.zeros((balloon_video.img_height, balloon_video.img_width, 3),numpy.uint8)
        image[:] = self.background_sky_colour_bgr
    
        # create large rectangle which will become the ground
        top_left = balloon_utils.rotate_xy(balloon_video.img_center_x-1000, balloon_video.img_center_y, -vehicle_roll) 
        top_right = balloon_utils.rotate_xy(balloon_video.img_center_x+1000, balloon_video.img_center_y, -vehicle_roll)
        bot_left = balloon_utils.rotate_xy(balloon_video.img_center_x-1000,balloon_video.img_center_y+1000, -vehicle_roll)
        bot_right = balloon_utils.rotate_xy(balloon_video.img_center_x+1000,balloon_video.img_center_y+1000, -vehicle_roll)
    
        # calculate vertical pixel shift
        pitch_pixel_shift = balloon_video.angle_to_pixels_y(vehicle_pitch)
    
        # add pitch adjustment
        top_left = balloon_utils.shift_pixels_down(top_left, pitch_pixel_shift)
        top_right = balloon_utils.shift_pixels_down(top_right, pitch_pixel_shift)
        bot_left = balloon_utils.shift_pixels_down(bot_left, pitch_pixel_shift)
        bot_right = balloon_utils.shift_pixels_down(bot_right, pitch_pixel_shift)
    
        # draw horizon
        box = numpy.array([top_left, top_right, bot_right, bot_left],numpy.int32)
        cv2.fillConvexPoly(image, box, self.background_ground_colour_bgr_scalar)
    
        return image

    # draw_fake_balloon - draws fake balloon in the frame at the specified roll, pitch and yaw angle
    # veh_pos : PositionVector holding the vehicle's offset from home
    # balloon_pos : PositionVector holding the balloon's offset from home
    # vehicle roll, pitch and yaw angles should be in radians
    def draw_fake_balloon(self, frame, veh_pos, balloon_pos, vehicle_roll, vehicle_pitch, vehicle_yaw):
        # calculate bearing to balloon
        bearing_to_balloon = PositionVector.get_bearing(veh_pos, balloon_pos)
        yaw_to_balloon = balloon_utils.wrap_PI(bearing_to_balloon-vehicle_yaw)

        # calculate earth frame pitch angle from vehicle to balloon
        pitch_to_balloon = vehicle_pitch + PositionVector.get_elevation(veh_pos, balloon_pos)

        #print "Fake Balloon Bearing:%f Pitch:%f Dist:%f" % (math.degrees(bearing_to_balloon), math.degrees(pitch_to_balloon), dist_to_balloon_xy)

        # calculate pixel position of balloon
        balloon_x = balloon_video.angle_to_pixels_x(yaw_to_balloon) + balloon_video.img_center_x
        balloon_y = balloon_video.angle_to_pixels_y(pitch_to_balloon) + balloon_video.img_center_y

        # calculate size of balloon in pixels from distance and size
        dist_to_balloon_xyz = PositionVector.get_distance_xyz(veh_pos, balloon_pos)
        balloon_radius = balloon_utils.get_pixels_from_distance(dist_to_balloon_xyz, balloon_finder.balloon_radius_expected)

        # store balloon radius
        self.last_balloon_radius = balloon_radius

        # draw balloon
        cv2.circle(frame,(balloon_x,balloon_y), balloon_radius, self.fake_balloon_colour_bgr_scalar, -1)

    # get_simulated_frame - returns an image of a simulated background and balloon based upon vehicle position, vehicle attitude and balloon position
    def get_simulated_frame(self, veh_pos, vehicle_roll, vehicle_pitch, vehicle_yaw):
        # get balloon position
        balloon_pos = PositionVector.get_from_location(self.fake_balloon_location)
        # get background
        sim_frame = self.get_background(vehicle_roll, vehicle_pitch)
        # draw balloon on background
        self.draw_fake_balloon(sim_frame, veh_pos, balloon_pos, vehicle_roll, vehicle_pitch, vehicle_yaw)
        return sim_frame

    # main - tests this class
    def main(self):

        # set home to tridge's home field (absolute alt = 270)
        PositionVector.set_home_location(LocationGlobal(-35.362938,149.165085,0))

        # calculate balloon position
        fake_balloon_pos = PositionVector.get_from_location(self.fake_balloon_location)

        # vehicle attitude and position
        veh_pos = PositionVector(0,0,fake_balloon_pos.z) # at home location
        veh_roll = math.radians(0)     # leaned right 10 deg
        veh_pitch = math.radians(0)     # pitched back at 0 deg
        veh_yaw = PositionVector.get_bearing(veh_pos,fake_balloon_pos)  # facing towards fake balloon

        # display positions from home
        print "Vehicle %s" % veh_pos
        print "Balloon %s" % fake_balloon_pos

        # generate simulated frame of balloon 10m north, 2m above vehicle
        img = self.get_simulated_frame(veh_pos, veh_roll, veh_pitch, veh_yaw)

        while(True):
            # move vehicle towards balloon
            veh_pos = veh_pos + (fake_balloon_pos - veh_pos) * 0.01

            # regenerate frame
            img = self.get_simulated_frame(veh_pos, veh_roll, veh_pitch, veh_yaw)

            # look for balloon in image using blob detector        
            found_in_image, xpos, ypos, size = balloon_finder.analyse_frame(img)

            # display actual vs real distance
            dist_actual = PositionVector.get_distance_xyz(veh_pos, fake_balloon_pos)
            dist_est = balloon_utils.get_distance_from_pixels(size, balloon_finder.balloon_radius_expected)
            print "Dist Est:%f  Act:%f   Size Est:%f  Act:%f" % (dist_est, dist_actual, size, self.last_balloon_radius) 

            # show image
            cv2.imshow("fake balloon", img)

            # wait for keypress 
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

        # destroy windows
        cv2.destroyAllWindows()

# declare global instance
balloon_sim  = BalloonSimulator()

# call main if this file is being run from the command line
if __name__ == "__main__":
    balloon_sim.main()
