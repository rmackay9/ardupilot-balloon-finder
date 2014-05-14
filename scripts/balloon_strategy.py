import time
from droneapi.lib import VehicleMode, Location
from find_balloon import get_camera, open_video_writer, analyse_frame, image_pos_to_angle, rotate_pos, add_artificial_horizon, pos_to_direction, get_distance_from_pixels, project_position 
from fake_balloon import get_simulated_frame, position_to_latlonalt
from math import degrees, radians

"""
This is an early guess at a top level controller that uses the DroneAPI and OpenCV magic
to try and win the challenge.

General approach:
* A state machine will need to know if we are in the portion of the route where we should even
be looking for balloons (by looking at vehicle mode == AUTO and between wpt #x and #y?)
* Periodically grab openCV frames and ask the image analysis function if it thinks there
is a balloon there and if so - what direction should we go.
* Initially we use current vehicle position + orientation + camera orentation to do a GUIDED
goto for a wpt beyond where the vehicle should go.
* If our altitude gets too low, or we get too far from the 'bee-line' path, abandon looking for
this particular balloon (to prevent crashing or wasting too much time looking for balloons)
* Until vehicle enters auto mode the strategy will only look for balloons (and generate
log messages
* If there was a mavlink message for telling the vehicle to shoot for a particular heading/altitude
rather than just going to wpts that might be a good optimization

To run this module:
* Run mavproxy.py with the correct options to connect to your vehicle
* module load api
* api start balloon-strategy.py

(Once tested we can put these directives into a mavinit.scr file and mavproxy will load/run
    this code automatically)

"""

use_simulator = False
#use_simulator = True

class BalloonStrategy(object):
    def __init__(self):
        # First get an instance of the API endpoint (the connect via web case will be similar)
        self.api = local_connect()

        # Our vehicle (we assume the user is trying to control the virst vehicle attached to the GCS)
        self.vehicle = self.api.get_vehicles()[0]

        # Our guess of the balloon location (if we have one)
        self.balloon_loc = None

        # The module only prints log messages unless the vehicle is in GUIDED mode (for testing).
        # Once this seems to work reasonablly well change self.debug to False and then it will
        # actually _enter_ guided mode when it thinks it sees a balloon
        self.debug = True

        self.min_wpt = 1 # If the vehicle is in its AUTO mission, we only look for balloons between these two wpts
        self.max_wpt = 4

        if not use_simulator:
            self.camera = get_camera()
        self.writer = open_video_writer()

    def get_frame(self):
        if use_simulator:
            veh_pos = (self.vehicle.location.lat,self.vehicle.location.lon,self.vehicle.location.alt)
            frame = get_simulated_frame(veh_pos,self.vehicle.attitude.roll,self.vehicle.attitude.pitch,self.vehicle.attitude.yaw)
        else:
            _, frame = self.camera.read()
        return frame

    def analyze_image(self):

        # capture vehicle's roll, pitch, yaw
        roll_in_radians = self.vehicle.attitude.roll
        pitch_in_radians = self.vehicle.attitude.pitch
        yaw_in_radians = self.vehicle.attitude.yaw

        f = self.get_frame()

        # FIXME - analyze the image to get a score indicating likelyhood there is a balloon and if it
        # is there the x & y position in frame of the largest balloon
        # FIXME - check if the balloon gets larger if we think we are approaching it
        found_in_image, xpos, ypos, size = analyse_frame(f)

        # add artificial horizon
        add_artificial_horizon(f, roll_in_radians, pitch_in_radians)
            
        if found_in_image:
            vehicle_pos = self.vehicle.location
            vehicle_attitude = self.vehicle.attitude

            # convert x, y position to pitch and yaw direction (in degrees)
            pitch_dir, yaw_dir = pos_to_direction(xpos, ypos, roll_in_radians, pitch_in_radians, yaw_in_radians)

            # get distance
            balloon_distance = get_distance_from_pixels(size)

            # debug
            #print "Balloon found at heading %f, and %f degrees up, dist:%f meters" % (yaw_dir, pitch_dir, balloon_distance)

            # calculate expected position
            pos_offset = project_position(radians(pitch_dir),radians(yaw_dir),balloon_distance)

            # FIXME - do math based on current vehicle loc and the x,y frame position
            (lat_offset, lon_offset, alt_offset) = position_to_latlonalt(pos_offset)

            target_pos = Location(vehicle_pos.lat + lat_offset, vehicle_pos.lon + lon_offset, vehicle_pos.alt + alt_offset, vehicle_pos.is_relative)
            #print "Balloon found at %s" % target_pos

            # FIXME - check if vehicle altitude is too low
            # FIXME - check if we are too far from the desired flightplan

            self.balloon_loc = target_pos

        # save image for debugging later
        self.writer.write(f)

    def goto_balloon(self):
        dest = self.balloon_loc
        if dest is None:
            print "No balloon found"

        mode = self.vehicle.mode.name

        if mode == "AUTO" and not self.debug:
            VehicleMode("GUIDED")
        elif mode != "GUIDED":
            print "Not driving to %s" % dest
            return

        print "Going to: %s" % dest
        # A better implementation would only send new waypoints if the position had changed significantly
        vehicle.commands.goto(dest)
        vehicle.flush()

    def run(self):
        while not self.api.exit:
            self.analyze_image()
            self.goto_balloon()

            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(0.05)
        self.camera.release()

strat = BalloonStrategy()
strat.run()

