import math
from balloon_video import balloon_video

"""
This file include utility functions for the balloon_finder project
"""

#
# image related helper function
#

# shift_pixels_down - shifts point down by "shift pixels" 
def shift_pixels_down((x,y),shift_pixels):
    return (x, y+shift_pixels)

# rotate_xy - rotates an x,y pixel position around the center of the image by the specified angle (in radians)
# x and y : position integer or floats holding the pixel location as an offset from center or angle offset from center
# angle : float holding the angle (in radians) to rotate the position.  +ve = rotate clockwise, -ve = rotate counter clockwise
# returns x,y rotated by the angle
def rotate_xy(x, y, angle):
    cos_ang = math.cos(angle)
    sin_ang = math.sin(angle)
    x_centered = x - balloon_video.img_width
    y_centered = y - balloon_video.img_height
    x_rotated = x * cos_ang - y * sin_ang
    y_rotated = x * sin_ang + y * cos_ang
    return x_rotated, y_rotated

# get_distance_from_pixels - returns distance to balloon in meters given number of pixels in image and expected 0.5m radius
#    size_in_pizels : diameter or radius of the object on the image (in pixels)
#    actual_size : diameter or radius of the object in meters
def get_distance_from_pixels(size_in_pixels, actual_size):
    # avoid divide by zero by returning 9999.9 meters for zero sized object 
    if (size_in_pixels == 0):
        return 9999.9
    # convert num_pixels to angular size
    return actual_size / balloon_video.pixels_to_angle_x(size_in_pixels)

# get_pixels_from_distance - returns object diameter or radius in pixels for a given distance
#    distance : distance from object in meters
#    actual_size: diameter or radius of the object in meters
def get_pixels_from_distance(distance, actual_size):
    return balloon_video.angle_to_pixels_x(actual_size / distance)

# wrap_PI - wraps value between -2*PI ~ +2*PI (i.e. -360 ~ +360 degrees) down to -PI ~ PI (i.e. -180 ~ +180 degrees)
#    angle should be in radians
def wrap_PI(angle):
    if (angle > math.pi):
        return (angle - (math.pi * 2.0))
    if (angle < -math.pi):
        return (angle + (math.pi * 2.0))
    return angle
