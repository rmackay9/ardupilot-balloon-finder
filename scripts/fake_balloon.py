#
# fake_ballon.py - creates an imagine of a balloon given a vehicle position, vehicle attitude and balloon position 
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
import balloon_config
from find_balloon import rotate_pos, img_width, img_height, img_center_x, img_center_y, cam_hfov, cam_vfov

# fake balloon location
fake_balloon_latlonalt = (balloon_config.config.get_float('fake-balloon', 'lat',-35.362938),
                          balloon_config.config.get_float('fake-balloon', 'lon',149.164980),
                          balloon_config.config.get_float('fake-balloon', 'alt',1))

# fake home location - tridge's home field (absolute alt = 270)
home_latlonalt = (balloon_config.config.get_float('fake-balloon', 'home-lat',-35.362938),
                  balloon_config.config.get_float('fake-balloon', 'home-lon',149.165085),
                  balloon_config.config.get_float('fake-balloon', 'home-alt',0)) 

#home_pos_latlonalt = (40.072842,-105.230575,1586,0) # AVC home


# fake balloon colour in HSV palette
fake_balloon_h = balloon_config.config.get_integer('fake-balloon','h',175)
fake_balloon_s = balloon_config.config.get_integer('fake-balloon','s',165)
fake_balloon_v = balloon_config.config.get_integer('fake-balloon','v',127)

# fake balloon is 1m across
fake_balloon_radius = balloon_config.config.get_float('fake-balloon','radius_cm',0.5)

# fake ballon colour in BGR pallette
fake_ballon_colour_bgr = cv2.cvtColor(np.uint8([[[fake_balloon_h,fake_balloon_s,fake_balloon_v]]]),cv2.COLOR_HSV2BGR)
fake_ballon_colour_bgr_scalar = cv2.cv.Scalar(fake_ballon_colour_bgr.item(0), fake_ballon_colour_bgr.item(1), fake_ballon_colour_bgr.item(2))

# background sky colour
background_sky_colour_bgr_scalar = cv2.cv.Scalar(232, 228, 227)
background_ground_colour_bgr_scalar = cv2.cv.Scalar(87, 145, 158)

# conversion from lat/lon to meters from home
LATLON_TO_M = 111319.5
scale_down_lon = math.cos(math.radians(home_latlonalt[0]))

# latlon_to_position - converts a lat, lon, altitude to a distance from home in meters
#     To-Do: move this to a common python script
def latlonalt_to_position((lat,lon,alt)):
    # convert lat, lon to meters from home
    x = (lat - home_latlonalt[0]) * LATLON_TO_M
    y = (lon - home_latlonalt[1]) * LATLON_TO_M * scale_down_lon
    return (x,y,alt)

# position_to_latlonalt - converts a distance in meters into the equivalent lat and lon offsets
#     To-Do: move this to a common python script
#     Note: this function is confusing because it doesn't add the home lat, lon so it is different from the function above
def position_to_latlonalt((x,y,z)):
    # convert lat, lon to meters from home
    lat = x / LATLON_TO_M
    lon = y / (LATLON_TO_M * scale_down_lon)
    return (lat,lon,z)

def wrap_PI(angle_in_radians):
    if (angle_in_radians > math.pi):
        return (angle_in_radians - (math.pi * 2.0))
    if (angle_in_radians < -math.pi):
        return (angle_in_radians + (math.pi * 2.0))
    return angle_in_radians

# fake balloon position as an offset in meters from home
#    1st element is lat.  I.e. +ve = north, -ve = south
#    2nd element is lon.  I.e. +ve = east, -ve = west
#    3rd element is alt.  I.e. +ve is above home, -ve is below
fake_balloon_pos = latlonalt_to_position(fake_balloon_latlonalt)

# angle_to_pixel_x - converts a horizontal angle (i.e. yaw) to a number of pixels 
def angle_to_pixel_x(angle_in_radians):
    return int(angle_in_radians * img_width / math.radians(cam_hfov))

# angle_to_pixel_y - converts a vertical angle (i.e. pitch) to a number of pixels 
def angle_to_pixel_y(angle_in_radians):
    return int(angle_in_radians * img_height / math.radians(cam_vfov))

# shift_to_center - return pixel position to center 
def shift_to_center((x,y)):
    return (x+img_center_x, y+img_center_y)

# shift_down - shifts point down by "shift pixels" 
def shift_down((x,y),shift_pixels):
    return (x, y+shift_pixels)

# get_bearing - returns bearing from origin to destination
def get_bearing((origin_x,origin_y),(destination_x,destination_y)):
    bearing = math.radians(90) + math.atan2(-(destination_x-origin_x), destination_y-origin_y);
    if bearing < 0:
        bearing += math.radians(360);
    return bearing

# get_balloon_radius - returns balloon radius for a given distance
def get_balloon_radius(distance_in_m):
    return angle_to_pixel_x(fake_balloon_radius / distance_in_m)

# get_background - returns a background given a roll and pitch angle
def get_background(roll_in_radians, pitch_in_radians):
    # create empty image first
    image = np.zeros((img_height, img_width, 3),np.uint8)
    image[:] = (232, 228, 227)

    # calculate horizon
    top_left = (-1000,0)
    top_right = (1000,0)
    bot_left = (-1000,1000)
    bot_right = (1000,1000)

    # rotate horizon and shift to center
    top_left_rot = shift_to_center(rotate_pos(top_left[0],top_left[1], -roll_in_radians))
    top_right_rot = shift_to_center(rotate_pos(top_right[0],top_right[1], -roll_in_radians))
    bot_left_rot = shift_to_center(rotate_pos(bot_left[0],bot_left[1], -roll_in_radians))
    bot_right_rot = shift_to_center(rotate_pos(bot_right[0],bot_right[1], -roll_in_radians))

    # calculate vertical pixel shift
    pitch_pixel_shift = angle_to_pixel_y(pitch_in_radians)

    # add pitch adjustment
    top_left_rot = shift_down(top_left_rot,pitch_pixel_shift)
    top_right_rot = shift_down(top_right_rot,pitch_pixel_shift)
    bot_left_rot = shift_down(bot_left_rot,pitch_pixel_shift)
    bot_right_rot = shift_down(bot_right_rot,pitch_pixel_shift)

    # draw horizon
    box = np.array([top_left_rot,top_right_rot,bot_right_rot,bot_left_rot],np.int32)
    cv2.fillConvexPoly(image, box, background_ground_colour_bgr_scalar)

    return image

# draw_fake_balloon_rpy - draws fake balloon in the frame at the specified roll, pitch and yaw angle
def draw_fake_balloon_rpy(frame,(vehicle_lat,vehicle_lon,vehicle_alt), (balloon_lat,balloon_lon,balloon_alt), vehicle_roll_in_radians, vehicle_pitch_in_radians, vehicle_yaw_in_radians):
    dist_to_balloon_xy = math.sqrt(math.pow(balloon_lat-vehicle_lat,2)+math.pow(balloon_lon-vehicle_lon,2))
    bearing_to_balloon = get_bearing((vehicle_lat,vehicle_lon),(balloon_lat,balloon_lon))
    if dist_to_balloon_xy == 0 and balloon_alt == vehicle_alt:
        pitch_to_balloon = vehicle_pitch_in_radians
    else:
        pitch_to_balloon = -math.atan2(balloon_alt-vehicle_alt,dist_to_balloon_xy) + vehicle_pitch_in_radians;
    yaw_to_balloon = wrap_PI(bearing_to_balloon-vehicle_yaw_in_radians)
    #print "Fake Balloon Bearing:%f Pitch:%f Dist:%f" % (math.degrees(bearing_to_balloon), math.degrees(pitch_to_balloon), dist_to_balloon_xy)
    # calculate pixel position of balloon
    balloon_x = angle_to_pixel_x(yaw_to_balloon)
    balloon_y = angle_to_pixel_y(pitch_to_balloon)
    (balloon_x,balloon_y) = shift_to_center((balloon_x,balloon_y))
    balloon_radius = get_balloon_radius(dist_to_balloon_xy)
    # draw circle
    cv2.circle(frame,(balloon_x,balloon_y),balloon_radius,fake_ballon_colour_bgr_scalar,-1)

# get_simulated_frame - returns an image of a simulated background and balloon based upon vehicle position, vehicle attitude and balloon position
def get_simulated_frame((vehicle_lat,vehicle_lon,vehicle_alt), vehicle_roll_in_radians, vehicle_pitch_in_radians, vehicle_yaw_in_radians):
    # convert vehicle position to meters from home
    veh_pos = latlonalt_to_position((vehicle_lat,vehicle_lon,vehicle_alt))
    # get background
    sim_frame = get_background(vehicle_roll_in_radians,vehicle_pitch_in_radians)
    # draw balloon on background
    draw_fake_balloon_rpy(sim_frame,veh_pos,fake_balloon_pos,vehicle_roll_in_radians,vehicle_pitch_in_radians,vehicle_yaw_in_radians)
    return sim_frame

def main():

    # vehicle attitude
    veh_roll = math.radians(0);
    veh_pitch = math.radians(0);
    veh_yaw = math.radians(270);

    veh_pos = home_latlonalt
    # generate simulated frame of balloon 10m north, 2m above vehicle
    img = get_simulated_frame(veh_pos,veh_roll,veh_pitch,veh_yaw)

    while(True):
        #cv2.cv.ShowImage("fake balloon", background)
        cv2.imshow("fake balloon", img)

        # wait for keypress 
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    
    # destroy windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
