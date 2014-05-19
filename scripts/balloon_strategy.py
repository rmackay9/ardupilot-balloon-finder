import time
from droneapi.lib import VehicleMode, Location
import balloon_config
from balloon_utils import get_distance, filter_position
from find_balloon import get_camera, open_video_writer, analyse_frame, analyse_frame_for_blob, image_pos_to_angle, rotate_pos, add_artificial_horizon, pos_to_direction, get_distance_from_pixels, project_position 
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

class BalloonStrategy(object):
    def __init__(self):

        # First get an instance of the API endpoint (the connect via web case will be similar)
        self.api = local_connect()

        # Our vehicle (we assume the user is trying to control the virst vehicle attached to the GCS)
        self.vehicle = self.api.get_vehicles()[0]

        # Our guess of the balloon location (if we have one)
        self.balloon_loc = None

        # target location we are flying to (if we have one)
        self.guided_target = None

        # time the target balloon was last spotted
        self.last_spotted_time = 0

        # if we lose sight of a balloon for this many seconds we will consider it lost and give up on the search
        self.lost_sight_timeout = 3

        # The module only prints log messages unless the vehicle is in GUIDED mode (for testing).
        # Once this seems to work reasonablly well change self.debug to False and then it will
        # actually _enter_ guided mode when it thinks it sees a balloon
        self.debug = balloon_config.config.get_boolean('general','debug',True)

        # use the simulator to generate fake balloon images
        self.use_simulator = balloon_config.config.get_boolean('general','simulate',True)

        self.min_wpt = 1 # If the vehicle is in its AUTO mission, we only look for balloons between these two wpts
        self.max_wpt = 4

        if not self.use_simulator:
            self.camera = get_camera()
        self.writer = open_video_writer()

    def get_frame(self):
        if self.use_simulator:
            veh_pos = (self.vehicle.location.lat,self.vehicle.location.lon,self.vehicle.location.alt)
            frame = get_simulated_frame(veh_pos,self.vehicle.attitude.roll,self.vehicle.attitude.pitch,self.vehicle.attitude.yaw)
        else:
            _, frame = self.camera.read()
        return frame

    def analyze_image(self):

        # record time
        now = time.time()

        # capture vehicle position and attitude
        vehicle_pos = self.vehicle.location
        vehicle_attitude = self.vehicle.attitude

        # get new image from camera
        f = self.get_frame()

        # FIXME - analyze the image to get a score indicating likelyhood there is a balloon and if it
        # is there the x & y position in frame of the largest balloon
        # FIXME - check if the balloon gets larger if we think we are approaching it
        #found_in_image, xpos, ypos, size = analyse_frame(f)
        found_in_image, xpos, ypos, size = analyse_frame_for_blob(f)

        # add artificial horizon
        add_artificial_horizon(f, vehicle_attitude.roll, vehicle_attitude.pitch)

        if found_in_image:
            # record time balloon was found
            self.last_spotted_time = now

            # convert x, y position to pitch and yaw direction (in degrees)
            pitch_dir, yaw_dir = pos_to_direction(xpos, ypos, vehicle_attitude.roll, vehicle_attitude.pitch, vehicle_attitude.yaw)

            # get distance
            balloon_distance = get_distance_from_pixels(size)

            # debug
            if self.debug:
                print "Balloon found at heading %f, and %f degrees up, dist:%f meters" % (yaw_dir, pitch_dir, balloon_distance)

            # calculate expected position
            pos_offset = project_position(radians(pitch_dir),radians(yaw_dir),balloon_distance)

            # convert offset in meters to lat/lon
            (lat_offset, lon_offset, alt_offset) = position_to_latlonalt(pos_offset)

            # balloon location is vehicle location + offset
            self.balloon_loc = Location(vehicle_pos.lat + lat_offset, vehicle_pos.lon + lon_offset, vehicle_pos.alt + alt_offset, vehicle_pos.is_relative)
        else:
            # indicate that we did not see a balloon
            self.balloon_loc = None

        # save image for debugging later
        self.writer.write(f)

    def goto_balloon(self):

        # exit immediately if not in guided mode
        if self.vehicle.mode.name != "GUIDED":
            if self.debug:
                print "Not in Guided"
            return

        # balloon to control whether we update target location to autopilot
        update_target = False

        # get current time
        now = time.time()

        # if we have not seen a balloon
        if self.balloon_loc is None:

            # if we do not have a target
            if self.guided_target is None:
                # there is nothing to do but wait for a balloon to appear
                # To-Do: add search logic here?
                return;

            else:
                # if we are within 2m of the last place we saw the balloon but don't see the balloon we have probably popped it (return control to autopilot)
                if get_distance(self.vehicle.location, self.guided_target) < 2:
                    if self.debug:
                        print "Lost Balloon, Think we popped it"
                    self.complete()
                    return

                # if we have not seen the balloon in some time, give up on it and maybe start searching again
                if now - self.last_spotted_time > self.lost_sight_timeout:
                    if self.debug:
                        print "Lost Balloon, Giving up"
                    self.guided_target = None
        else:
            # we have seen a balloon

            # if we have never seen the balloon update our target to the balloon position
            if self.guided_target is None:
                self.guided_target = self.balloon_loc
                update_target = True
                if self.debug:
                        print "Found Balloon for the first time"

            else:
                # get distance the target has moved
                distance_target_moved = get_distance(self.guided_target, self.balloon_loc)

                # if it has moved less than 50cm ignore the change
                if distance_target_moved > 0.5:

                    #if it has moved less than 20m we guess it is the same balloon and move the position estimate towards the new reading
                    if distance_target_moved < 20:
                        filter_position(self.guided_target,self.balloon_loc,0.1)
                        update_target = True
                        if self.debug:
                            print "Moved target a little: %f" % (distance_target_moved * 0.1)

                    #if it has moved more than 20m we guess it is a different balloon
                    else:
                        # get distance to the target
                        distance_to_target = get_distance(self.vehicle.location, self.guided_target)

                        #if we've reached the position we have probably popped the 1st balloon and are now seeing a different balloon
                        if distance_to_target < 2:
                            if self.debug:
                                print "Found different ball at %f meters" % distance_to_target
                            self.complete()
                            return
                else:
                    # balloon moved less than 50cm
                    if self.debug:
                        print "Balloon didn't move much"

        # FIXME - check if vehicle altitude is too low
        # FIXME - check if we are too far from the desired flightplan

        # send new target to the autopilot
        if not self.guided_target is None:
            if update_target:
                self.vehicle.commands.goto(self.guided_target)
                self.vehicle.flush()
                if self.debug:
                    print "Going to: %s" % self.guided_target
        else:
            if self.debug:
                    print "No target"

    def run(self):
        while not self.api.exit:
            # To-Do: if our overall timeout has elapsed return control to autopilot

            # look for balloon in image
            self.analyze_image()

            # move towards balloon
            self.goto_balloon()

            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(0.05)
        self.camera.release()

    # complete - balloon strategy has somehow completed so return control to the autopilot
    def complete(self):
        # reset all variables
        self.balloon_loc = None
        self.guided_target = None
        self.last_spotted_time = 0

        # if in GUIDED mode switch back to LOITER
        if self.vehicle.mode.name == "GUIDED":
            self.vehicle.mode = VehicleMode("LOITER")
            self.vehicle.flush()
        return

strat = BalloonStrategy()
strat.run()

