import time
import math
from pymavlink import mavutil
from droneapi.lib import VehicleMode, Location
import balloon_config
from balloon_video import balloon_video
from balloon_utils import get_distance_from_pixels, wrap_PI
from position_vector import PositionVector
from find_balloon import balloon_finder
from fake_balloon import balloon_sim
import pid

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

        # search variables
        self.searching = False
        self.search_complete = False
        self.search_closest_balloon = None
        self.search_start_time = None
        self.search_start_heading = None
        self.search_heading_change = None

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
        self.last_target_update = time.time()   # time of the last target update sent to the flight controller
        
        # velocity control variables
        self.balloon_vel = None             # velocity vector to balloon
        self.guided_target_vel = None       # guided mode's target velocity

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

        if not self.use_simulator:
            self.camera = get_camera()
        self.writer = balloon_video.open_video_writer()

        self.search_target_heading = None

        # horizontal velocity pid controller.  maximum effect is 10 degree lean
        self.pid_xy = pid.pid(2.0, 0.0, 0.0, math.radians(10))

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

            # ensure the vehicle's position is known
            if self.vehicle.location is None:
                return False
            if self.vehicle.location.lat is None or self.vehicle.location.lon is None or self.vehicle.location.alt is None:
                return False

            # download the vehicle waypoints if we don't have them already
            if self.mission_cmds is None:
                self.fetch_mission()
                return False

            # get the home lat and lon
            home_lat = self.mission_cmds[0].x
            home_lon = self.mission_cmds[0].y

            # sanity check the home position
            if home_lat is None or home_lon is None:
                return False

            # sanity check again and set home position
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
                if not self.controlling_vehicle:
                    self.controlling_vehicle = True
                    self.start_search()
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
                    if not self.controlling_vehicle:
                        self.controlling_vehicle = True
                        self.start_search()
                    return    
        
            # if we got here then we are not in control
            self.controlling_vehicle = False

    # condition_yaw - send condition_yaw mavlink command to vehicle so it points at specified heading (in degrees)
    def condition_yaw(self, heading):
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                     0,     # sequence
                                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
                                                     2, # current - set to 2 to make it a guided command
                                                     0, # auto continue
                                                     heading, 0, 0, 0, 0, 0, 0) # param 1 ~ 7
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # send_nav_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def send_nav_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                     0,     # sequence
                                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     91, # command id, replace with mavutil.mavlink.MAV_CMD_NAV_VELOCITY
                                                     2, # current - set to 2 to make it a guided command
                                                     0, # auto continue
                                                     0,                         # frame (unused)
                                                     0, 0, 0,                   # params 1 ~ 4 (unused)
                                                     velocity_x, velocity_y, velocity_z) # params 5 ~ 7
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # advance_current_cmd - ask vehicle to advance to next command (i.e. abort current command)
    def advance_current_cmd(self):

        # exit immediately if we are not in AUTO mode or not controlling the vehicle
        if not self.vehicle.mode.name == "AUTO" or not self.controlling_vehicle:
            return

        # download the vehicle waypoints if we don't have them already
        if self.mission_cmds is None:
            self.fetch_mission()

        # get active command
        active_command = self.vehicle.commands.next

        # ensure there is one more command at least
        if (self.vehicle.commands.count > active_command):
            # create the MISSION_SET_CURRENT command
            msg = self.vehicle.message_factory.mission_set_current_encode(0, 0, active_command+1) # target system, target component, sequence
            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
        else:
            print "Failed to advance command"

    # start_search - start search for balloon
    def start_search(self):
        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle:
            return
        # initialise search variables
        self.searching = True
        self.search_complete = False
        self.search_closest_balloon = None
        self.search_start_time = time.time()
        self.search_start_heading = self.vehicle.attitude.yaw
        self.search_target_heading = self.search_start_heading
        self.search_total_angle = 0

    # search - spin vehicle looking for balloon
    def search_for_balloon(self):

        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle or not self.searching or self.vehicle_pos is None:
            return

        # check if we're still searching for balloon
        if not self.search_complete:
            # check if we have seen a balloon
            if not self.balloon_pos is None:
                # if this is the first balloon we've seen store it's position as the closest
                if self.search_closest_balloon is None:
                    self.search_closest_balloon = self.balloon_pos
                else:
                    # check distance to new balloon vs previous closest balloon and store if closer
                    dist_prev = PositionVector.get_distance_xyz(self.vehicle_pos, self.search_closest_balloon)
                    dist_new = PositionVector.get_distance_xyz(self.vehicle_pos, self.balloon_pos)
                    if dist_new < dist_prev:
                        self.search_closest_balloon = self.balloon_pos

            # check yaw is close to target
            if math.fabs(wrap_PI(self.vehicle.attitude.yaw - self.search_target_heading)) < math.radians(20):
                # increase yaw target
                self.search_target_heading = self.search_target_heading - math.radians(10)
                self.search_total_angle = self.search_total_angle + math.radians(10)
                # send yaw heading
                self.condition_yaw(math.degrees(self.search_target_heading))

                # end search if we've gone all the way around
                if self.search_total_angle >= math.radians(360):
                    # record search as complete
                    self.search_complete = True

                    # if we never saw a balloon then just complete (return control to user or mission)
                    if self.search_closest_balloon is None:
                        self.searching = False
                        self.complete()
                    else:
                        # set target heading to closest balloon
                        self.search_target_heading = PositionVector.get_bearing(self.vehicle_pos, self.search_closest_balloon)
                        # send yaw heading
                        self.condition_yaw(math.degrees(self.search_target_heading))

        # we have completed search and are spinning back towards closest balloon
        else:
            # check yaw is close to target
            if math.fabs(wrap_PI(self.vehicle.attitude.yaw - self.search_target_heading)) < math.radians(5):
                # if the balloon is visible then stop searching
                if not self.balloon_pos is None:
                    self.searching = False
                else:
                    # we somehow haven't found the balloon when we've turned back to find it so begin search again
                    self.start_search()

    def get_frame(self):
        if self.use_simulator:
            veh_pos = PositionVector.get_from_location(self.vehicle.location)
            frame = balloon_sim.get_simulated_frame(veh_pos, self.vehicle.attitude.roll, self.vehicle.attitude.pitch, self.vehicle.attitude.yaw)
        else:
            _, frame = self.camera.read()
        return frame

    # get image from camera and look for balloon
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

            # calculate change in yaw error since we began the search
            yaw_error = 0
            if not self.search_target_heading is None:
                yaw_error = wrap_PI(math.radians(yaw_dir) - self.search_target_heading)
            else:
                self.pid_xy.reset_I()
            
            # get distance
            balloon_distance = get_distance_from_pixels(size, balloon_finder.balloon_radius_expected)

            # debug
            #if self.debug:
            #    print "Balloon found at heading %f, and %f degrees up, dist:%f meters" % (yaw_dir, pitch_dir, balloon_distance)

            # updated estimated balloon position
            self.balloon_pos = balloon_finder.project_position(self.vehicle_pos, math.radians(pitch_dir), math.radians(yaw_dir), balloon_distance)

            # get speed towards balloon between 1m/s and 5m/s
            speed = min(balloon_distance,4)
            speed = max(speed,1)

            # calculate yaw error
            yaw_pid_error = self.pid_xy.get_pid(yaw_error, self.pid_xy.get_dt(0.5))

            # calculate velocity vector towards balloon
            self.balloon_vel = balloon_finder.get_ef_velocity_vector(math.radians(pitch_dir), math.radians(yaw_dir)+yaw_pid_error, speed)
        else:
            # indicate that we did not see a balloon
            self.balloon_pos = None
            self.balloon_vel = None

        # save image for debugging later
        self.writer.write(f)

    def goto_balloon(self):

        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle:
            return

        # get current time
        now = time.time()

        # exit immediately if it's been too soon since the last update
        if (now - self.last_target_update) < 2.0:
            return;

        # balloon to control whether we update target location to autopilot
        update_target = False

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
            self.last_target_update = time.time()
            #if self.debug:
            #    print "Veh %s" % self.vehicle.location
            #    print "Going to: %s" % self.guided_target_loc

    def speed_to_balloon(self):

        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle:
            return

        # get current time
        now = time.time()

        # exit immediately if it's been too soon since the last update
        if (now - self.last_target_update) < 1.0:
            return;

        # balloon to control whether we update target velocity to autopilot
        update_target = False

        # if we have not seen the balloon in our most recent image
        if self.balloon_vel is None:
            # if we do not have a target there is nothing to do but wait for a balloon to appear
            if self.guided_target_vel is None:
                # To-Do: add search logic here?
                return;

            # although we don't see the balloon we still have a target velocity towards the last place we saw it
            else:
                # if we have not seen the balloon in some time, give up on it
                if now - self.last_spotted_time > self.lost_sight_timeout:
                    if self.debug:
                        print "Lost Balloon, Giving up"
                    # clear out target position
                    self.guided_target_vel = None
                    # To-Do: stop vehicle moving and start searching again?
                    self.complete()

        # we have seen a balloon
        else:
            # if we have never seen the balloon update our target velocity to fly at the balloon
            if self.guided_target_vel is None:
                if self.debug:
                        print "Found Balloon for the first time"

            self.guided_target_vel = self.balloon_vel
            update_target = True

        # FIXME - check if vehicle altitude is too low
        # FIXME - check if we are too far from the desired flightplan

        # send new target to the autopilot
        if update_target:
            self.send_nav_velocity(self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2])
            self.vehicle.flush()
            self.last_target_update = time.time()

    def run(self):
        while not self.api.exit:

            # only process images once home has been initialised
            if self.check_home():
    
                # check if we are controlling the vehicle
                self.check_status()

                # look for balloon in image
                self.analyze_image()
    
                # search or move towards balloon
                if self.searching:
                    # search for balloon
                    self.search_for_balloon()
                else:
                    # move towards balloon
                    self.speed_to_balloon()
    
            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(0.05)

        self.camera.release()

    # complete - balloon strategy has somehow completed so return control to the autopilot
    def complete(self):
        # debug 
        if self.debug:
            print "Complete!"

        # set target velocity to zero
        self.send_nav_velocity(0,0,0)

        # record that we are not in control of vehicle
        # To-Do: this could be reset back to True if check runs too soon after this
        self.controlling_vehicle = False

        # reset all variables
        self.balloon_loc = None
        self.guided_target_loc = None
        self.guided_target_pos = None
        self.last_spotted_time = 0

        # if in GUIDED mode switch back to LOITER
        if self.vehicle.mode.name == "GUIDED":
            self.vehicle.mode = VehicleMode("LOITER")
            self.vehicle.flush()

        # if in AUTO move to next command
        if self.vehicle.mode.name == "AUTO":
            self.advance_current_cmd();

        return

strat = BalloonStrategy()
strat.run()

