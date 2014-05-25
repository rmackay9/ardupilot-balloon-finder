import time
import math
from droneapi.lib import VehicleMode, Location
import balloon_config
from balloon_video import balloon_video
from balloon_utils import get_distance_from_pixels
from position_vector import PositionVector
from find_balloon import balloon_finder
from fake_balloon import balloon_sim

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

        # initialised flag
        self.home_initialised = False
        # timer to intermittently check for home position
        self.last_home_check = time.time()

        # vehicle mission
        self.mission_cmds = None

        # we are not in control of vehicle
        self.controlling_vehicle = False
        self.last_status_check = time.time()

        # vehicle position captured at time camera image was captured
        self.vehicle_pos = None

        # Our guess of the balloon position as an offset from home (if we have one)
        self.balloon_pos = None             # last estimated position

        # target location we are flying to (if we have one)
        self.guided_target_pos = None
        self.guided_target_loc = None

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
        self.writer = balloon_video.open_video_writer()

    # fetch_mission - fetch mission from flight controller
    def fetch_mission(self):
        # download the vehicle waypoints
        self.mission_cmds = self.vehicle.commands
        self.mission_cmds.download()
        self.mission_cmds.wait_valid()

    # check home - intermittently checks for changes to the home location
    def check_home(self):

        # return immediately if home has already been initialised
        if self.home_initialised:
            return True

        # check for home no more than once every two seconds
        if (time.time() - self.last_home_check > 2):

            # update that we have performed a status check
            self.last_home_check = time.time()

            # download the vehicle waypoints if we don't have them already
            if self.mission_cmds is None:
                self.fetch_mission()
                return

            # get the home lat and lon
            home_lat = self.mission_cmds[0].x
            home_lon = self.mission_cmds[0].y

            # sanity check the home position
            if home_lat <> 0 and home_lon <> 0:
                PositionVector.set_home_location(Location(home_lat,home_lon,0))
                self.home_initialised = True

            # To-Do: if we wish to have the same home position as the flight controller
            # we must download the home waypoint again whenever the vehicle is armed

        # return whether home has been initialised or not
        return self.home_initialised

    # check_status - poles vehicle' status to determine if we are in control of vehicle or not
    def check_status(self):

        # check status no more than once every two seconds
        if (time.time() - self.last_status_check > 2):

            # update that we have performed a status check
            self.last_status_check = time.time()

            # we are active in guided mode
            if self.vehicle.mode.name == "GUIDED":
                self.controlling_vehicle = True
                return

            # download the vehicle waypoints if we don't have them already
            # To-Do: do not load waypoints if vehicle is armed
            if self.mission_cmds is None:
                self.fetch_mission()
                return

            # Check for Auto mode and executing Nav-Guided command
            if self.vehicle.mode.name == "AUTO":

                # get active command number and mavlink id
                active_command = self.vehicle.commands.next
                active_command_id = self.vehicle.commands[active_command].command

                # ninety is the MAVLink id for Nav-Guided commands
                if active_command_id == 90:
                    self.controlling_vehicle = True
                    return    
        
            # if we got here then we are not in control
            self.controlling_vehicle = False

    def get_frame(self):
        if self.use_simulator:
            veh_pos = PositionVector.get_from_location(self.vehicle.location)
            frame = balloon_sim.get_simulated_frame(veh_pos, self.vehicle.attitude.roll, self.vehicle.attitude.pitch, self.vehicle.attitude.yaw)
        else:
            _, frame = self.camera.read()
        return frame

    def analyze_image(self):

        # record time
        now = time.time()

        # capture vehicle position and attitude
        self.vehicle_pos = PositionVector.get_from_location(self.vehicle.location)
        vehicle_attitude = self.vehicle.attitude

        # get new image from camera
        f = self.get_frame()

        # FIXME - analyze the image to get a score indicating likelihood there is a balloon and if it
        # FIXME - check if the balloon gets larger if we think we are approaching it

        # look for balloon in image using blob detector        
        found_in_image, xpos, ypos, size = balloon_finder.analyse_frame_for_blob(f)

        # add artificial horizon
        balloon_finder.add_artificial_horizon(f, vehicle_attitude.roll, vehicle_attitude.pitch)

        if found_in_image:
            # record time balloon was found
            self.last_spotted_time = now

            # convert x, y position to pitch and yaw direction (in degrees)
            pitch_dir, yaw_dir = balloon_finder.pixels_to_direction(xpos, ypos, vehicle_attitude.roll, vehicle_attitude.pitch, vehicle_attitude.yaw)

            # get distance
            balloon_distance = get_distance_from_pixels(size, balloon_finder.balloon_radius_expected)

            # debug
            #if self.debug:
            #    print "Balloon found at heading %f, and %f degrees up, dist:%f meters" % (yaw_dir, pitch_dir, balloon_distance)

            # updated estimated balloon position
            self.balloon_pos = balloon_finder.project_position(self.vehicle_pos, math.radians(pitch_dir),math.radians(yaw_dir),balloon_distance)
        else:
            # indicate that we did not see a balloon
            self.balloon_pos = None

        # save image for debugging later
        self.writer.write(f)

    def goto_balloon(self):

        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle:
            return

        # balloon to control whether we update target location to autopilot
        update_target = False

        # get current time
        now = time.time()

        # if we have not seen the balloon in our most recent image
        if self.balloon_pos is None:
            # if we do not have a target there is nothing to do but wait for a balloon to appear
            if self.guided_target_pos is None:
                # To-Do: add search logic here?
                return;

            # although we don't see the balloon we still have a target to the last place we saw it
            else:
                # if we are within 2m of the target we have probably popped it (return control to autopilot)
                if PositionVector.get_distance_xyz(self.vehicle_pos, self.guided_target_pos) < 2:
                    if self.debug:
                        print "Lost Balloon, Think we popped it"
                    self.complete()
                    return

                # if we have not seen the balloon in some time, give up on it
                if now - self.last_spotted_time > self.lost_sight_timeout:
                    if self.debug:
                        print "Lost Balloon, Giving up"
                    # clear out target position
                    self.guided_target_pos = None
                    self.guided_target_loc = None
                    # To-Do: stop vehicle moving and start searching again?
                    self.complete()

        # we have seen a balloon
        else:
            # if we have never seen the balloon update our target to the balloon position
            if self.guided_target_pos is None:
                self.guided_target_pos = self.balloon_pos
                self.guided_target_loc = self.guided_target_pos.get_location()
                update_target = True
                if self.debug:
                        print "Found Balloon for the first time"

            else:
                # get distance the balloon has apparently moved
                distance_balloon_moved = PositionVector.get_distance_xyz(self.guided_target_pos, self.balloon_pos)

                # if balloon has moved more than the radius of the balloon we adjust target
                if distance_balloon_moved > 0.5:
                    #if self.debug:
                    #    print "Dist Balloon Moved: %f" % distance_balloon_moved

                    # if balloon has apparently moved less than 20m we guess it is the same balloon
                    if distance_balloon_moved < 20:
                        # update the target position towards the balloon position
                        self.guided_target_pos = self.guided_target_pos + (self.balloon_pos - self.guided_target_pos) * 0.1

                        # convert the new target to a location
                        target_loc = self.guided_target_pos.get_location()

                        # if different from current target update flight controller target
                        dalt = math.fabs(target_loc.alt - self.guided_target_loc.alt)
                        if (target_loc.lat <> self.guided_target_loc.lat or target_loc.lon <> self.guided_target_loc.lon or dalt > 0.10):
                            self.guided_target_loc = target_loc
                            update_target = True
                            #if self.debug:
                            #    print "Moved target a little: %f" % (distance_balloon_moved * 0.1)

                    # if balloon has moved more than 20m we guess it is a different balloon
                    else:
                        # get distance to the target
                        distance_to_target = PositionVector.get_distance_xyz(self.vehicle_pos, self.guided_target_pos)

                        #if we've reached the position we have probably popped the 1st balloon and are now seeing a different balloon
                        if distance_to_target < 2:
                            if self.debug:
                                print "Found different ball at %f meters" % distance_to_target
                            # To-Do: try going for this new balloon?  Would need to re-check it's distance, altitude, etc
                            self.complete()
                            return

        # FIXME - check if vehicle altitude is too low
        # FIXME - check if we are too far from the desired flightplan

        # send new target to the autopilot
        if update_target:
            self.vehicle.commands.goto(self.guided_target_loc)
            self.vehicle.flush()
            if self.debug:
                print "Veh %s" % self.vehicle.location
                print "Going to: %s" % self.guided_target_loc

    def run(self):
        while not self.api.exit:

            # only process images once home has been initialised
            if self.check_home():
    
                # check if we are controlling the vehicle
                self.check_status()

                # look for balloon in image
                self.analyze_image()
    
                # move towards balloon
                self.goto_balloon()
    
            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(0.05)

        self.camera.release()

    # complete - balloon strategy has somehow completed so return control to the autopilot
    def complete(self):
        # debug 
        if self.debug:
            print "Complete!"
        # reset all variables
        self.balloon_loc = None
        self.guided_target_loc = None
        self.guided_target_pos = None
        self.last_spotted_time = 0

        # if in GUIDED mode switch back to LOITER
        if self.vehicle.mode.name == "GUIDED":
            self.vehicle.mode = VehicleMode("LOITER")
            self.vehicle.flush()
        return

strat = BalloonStrategy()
strat.run()

