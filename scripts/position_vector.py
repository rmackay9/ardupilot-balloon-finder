"""
PositionVector class : holds a 3 axis position offset from home in meters in NEU format
                       X = North-South with +ve = North
                       Y = West-East with +ve = West
                       Z = Altitude with +ve = Up

"""

import math
from dronekit import LocationGlobal

# global variables used by position vector class
posvec_home_location = LocationGlobal(0,0,0)
posvec_lon_scale = 1.0          # scaling used to account for shrinking distance between longitude lines as we move away from equator
posvec_latlon_to_m = 111319.5   # converts lat/lon to meters

class PositionVector(object):

    # define public members
    x = 0       # north-south direction.  +ve = north of home
    y = 0       # west-east direction, +ve = east of home
    z = 0       # vertical direction, +ve = above home

    def __init__(self,initial_x=0,initial_y=0,initial_z=0):
        # default config file
        self.x = initial_x
        self.y = initial_y
        self.z = initial_z

    # get_from_location - returns a position vector created from a location
    @classmethod
    def get_from_location(cls, location):
        ret = PositionVector(0,0,0)
        ret.set_from_location(location)
        return ret

    # __str__ - print position vector as string
    def __str__(self):
        return "Pos:X=%s,Y=%s,Z=%s" % (self.x, self.y, self.z)

    # __add__ - addition operator overload
    def __add__(self, other):
        return PositionVector(self.x + other.x, self.y + other.y, self.z + other.z)

    # __sub__ - subtraction operator overload
    def __sub__(self, other):
        return PositionVector(self.x - other.x, self.y - other.y, self.z - other.z)

    # __mul__ - multiply operator overload
    def __mul__(self, scalar):
        return PositionVector(self.x * scalar, self.y * scalar, self.z * scalar)

    # get_location - return the location (i.e. lat, lon, alt) of the position vector
    def get_location(self):
        dlat = self.x / posvec_latlon_to_m
        dlon = self.y / (posvec_latlon_to_m * posvec_lon_scale)
        return LocationGlobal(posvec_home_location.lat + dlat,posvec_home_location.lon + dlon,posvec_home_location.alt + self.z)

    # set_from_location - sets x,y,z offsets given a location object (i.e. lat, lon and alt)
    def set_from_location(self, location):
        # convert lat, lon to meters from home
        self.x = (location.lat - posvec_home_location.lat) * posvec_latlon_to_m
        self.y = (location.lon - posvec_home_location.lon) * posvec_latlon_to_m * posvec_lon_scale
        self.z = location.alt

    # set_home_location - sets home location used by all position vector instances
    @classmethod
    def set_home_location(cls, home_location):
        global posvec_home_location
        posvec_home_location = home_location
        PositionVector.update_lon_scale(posvec_home_location.lat)

    # get_home_location - returns home location used by all position vector instances
    @classmethod
    def get_home_location(cls):
        global posvec_home_location
        return posvec_home_location

    # get_distance_xy - returns horizontal distance in meters from one position to another
    @classmethod
    def get_distance_xy(cls, pos1, pos2):
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        return math.sqrt(dx**2+dy**2)

    # get_distance_xyz - returns distance in meters from one position to another
    @classmethod
    def get_distance_xyz(cls, pos1, pos2):
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        dz = pos2.z - pos1.z
        return math.sqrt(dx**2+dy**2+dz**2)

    # get_bearing - returns bearing from origin to destination in radians
    @classmethod
    def get_bearing(cls, origin, destination):
        # avoid error when origin and destination are exactly on top of each other
        if destination.x == origin.x and destination.y == origin.y:
            return 0
        bearing = math.radians(90) + math.atan2(-(destination.x-origin.x), destination.y-origin.y);
        if bearing < 0:
            bearing += math.radians(360);
        return bearing

    # get_elevation - returns an elevation in radians from origin to destination
    @classmethod
    def get_elevation(cls, origin, destination):
        # avoid error when origin and destination are exactly on top of each other
        if destination.x == origin.x and destination.y == origin.y and destination.z == origin.z:
            return 0

        # calculate distance to destination
        dist_xy = PositionVector.get_distance_xy(origin, destination)

        # calculate elevation
        elevation = -math.atan2(destination.z-origin.z, dist_xy)
        return elevation

    # get_lon_scale - return lon scaling factor to account for curvature of earth
    @classmethod
    def update_lon_scale(cls, lat):
        global posvec_lon_scale
        if lat <> 0:
            posvec_lon_scale = math.cos(math.radians(lat))

    # main - used to test the class
    def main(self):
        # set home position - to tridge's home field (this is just for testing anyway)
        PositionVector.set_home_location(LocationGlobal(-35.362938,149.165085,0))
        print "Home %s" % PositionVector.get_home_location()
        home_pos = PositionVector(0,0,0)
        print "Home %s" % home_pos

        # other position
        other_pos = PositionVector.get_from_location(PositionVector.get_home_location())
        print "Other %s" % other_pos

        # set vehicle to be 10m north, 10m east and 10m above home
        veh_pos = PositionVector(10,10,10)
        print "Vehicle %s" % veh_pos.get_location()
        print "Vehicle %s" % veh_pos
        print "Distance from home: %f" % PositionVector.get_distance_xyz(home_pos,veh_pos)


# run the main routine if this is file is called from the command line
if __name__ == "__main__":
    dummy = PositionVector(0,0,0)
    dummy.main()