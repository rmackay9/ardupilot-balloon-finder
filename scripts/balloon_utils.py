from droneapi.lib import Location
from math import cos, sin, sqrt, radians

"""
This file include utility functions for the balloon_finder project
"""

lon_scale = 0.0             # longitude scaling
LATLON_TO_M = 111319.5      # converts lat/lon to meters

# get_lon_scale - return lon scaling factor to account for curvature of earth
def get_lon_scale(lat):
    global lon_scale
    if lon_scale == 0:
        lon_scale = cos(radians(lat))
    return lon_scale

# get_distance - return distance in meters between two locations
def get_distance(location1, location2):
    dlat = location2.lat - location1.lat
    dlon = (location2.lon - location1.lon) * get_lon_scale(location1.lat)
    return sqrt(dlat**2 + dlon**2) * LATLON_TO_M;

# filter_position - move orig_loc towards new_loc
def filter_position(orig_loc, new_loc, factor):
    dlat = new_loc.lat - orig_loc.lat
    dlon = new_loc.lon - orig_loc.lon
    orig_loc.lat += dlat * factor
    orig_loc.lon += dlon * factor