import sys
import os
sys.path.insert(1, os.getcwd())

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
from attitude_history import AttitudeHistory

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

or

* execute linux_run_strategy.sh

Then arm it an takeoff as in mavproxy_sequence.txt
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

        # historical attitude
        self.att_hist = AttitudeHistory(self.vehicle, 2.0)
        self.attitude_delay = 0.0               # expected delay between image and attitude

        # search variables
        self.search_state = 0                   # 0 = not search, 1 = spinning and looking, 2 = rotating to best balloon and double checking, 3 = finalise yaw 
        self.search_start_heading = None        # initial heading of vehicle when search began
        self.search_target_heading = None       # the vehicle's current target heading (updated as vehicle spins during search)
        self.search_heading_change = None       # heading change (in radians) performed so far during search
        self.search_balloon_pos = None          # position (as an offset from home) of closest balloon (so far) during search
        self.search_balloon_heading = None      # earth-frame heading (in radians) from vehicle to closest balloon
        self.search_balloon_pitch_top = None        # earth-frame pitch (in radians) from vehicle to closest balloon
        self.search_balloon_distance = None     # distance (in meters) from vehicle to closest balloon
        self.targeting_start_time = 0           # time vehicle first pointed towards final target (used with delay_time below)
        self.targeting_delay_time = balloon_config.config.get_float('general','SEARCH_TARGET_DELAY',2.0)    # time vehicle waits after pointing towards ballon and before heading towards it
        self.search_yaw_speed = balloon_config.config.get_float('general','SEARCH_YAW_SPEED',5.0) 

        # vehicle mission
        self.mission_cmds = None
        self.mission_alt_min = 0                # min altitude from NAV_GUIDED mission command (we ignore balloons below this altitude).  "0" means no limit
        self.mission_alt_max = 0                # max altitude from NAV_GUIDED mission command (we ignore balloons above this altitude).  "0" means no limit
        self.mission_distance_max = 0           # max distance from NAV_GUIDED mission command (we ignore balloons further than this distance).  "0" means no limit

        # we are not in control of vehicle
        self.controlling_vehicle = False

        # vehicle position captured at time camera image was captured
        self.vehicle_pos = None

        # balloon direction and position estimate from latest call to analyse_image
        self.balloon_found = False
        self.balloon_pitch = None
        self.balloon_pitch_top = None           # earth-frame pitch (in radians) from vehicle to top of closest balloon
        self.balloon_heading = None
        self.balloon_distance = None
        self.balloon_pos = None             # last estimated position as an offset from home

        # time of the last target update sent to the flight controller
        self.guided_last_update = time.time()

        # latest velocity target sent to flight controller
        self.guided_target_vel = None

        # time the target balloon was last spotted
        self.last_spotted_time = 0

        # if we lose sight of a balloon for this many seconds we will consider it lost and give up on the search
        self.lost_sight_timeout = 3

        # The module only prints log messages unless the vehicle is in GUIDED mode (for testing).
        # Once this seems to work reasonablly well change self.debug to False and then it will
        # actually _enter_ guided mode when it thinks it sees a balloon
        self.debug = balloon_config.config.get_boolean('general','debug',True)

        # use the simulator to generate fake balloon images
        self.use_simulator = balloon_config.config.get_boolean('general','simulate',False)

        # start background image grabber
        if not self.use_simulator:
            balloon_video.start_background_capture()

        # initialise video writer
        self.writer = None

        # horizontal velocity pid controller.  maximum effect is 10 degree lean
        xy_p = balloon_config.config.get_float('general','VEL_XY_P',1.0)
        xy_i = balloon_config.config.get_float('general','VEL_XY_I',0.0)
        xy_d = balloon_config.config.get_float('general','VEL_XY_D',0.0)
        xy_imax = balloon_config.config.get_float('general','VEL_XY_IMAX',10.0)
        self.vel_xy_pid = pid.pid(xy_p, xy_i, xy_d, math.radians(xy_imax))

        # vertical velocity pid controller.  maximum effect is 10 degree lean
        z_p = balloon_config.config.get_float('general','VEL_Z_P',2.5)
        z_i = balloon_config.config.get_float('general','VEL_Z_I',0.0)
        z_d = balloon_config.config.get_float('general','VEL_Z_D',0.0)
        z_imax = balloon_config.config.get_float('general','VEL_IMAX',10.0)
        self.vel_z_pid = pid.pid(z_p, z_i, z_d, math.radians(z_imax))

        # velocity controller min and max speed
        self.vel_speed_min = balloon_config.config.get_float('general','VEL_SPEED_MIN',1.0)
        self.vel_speed_max = balloon_config.config.get_float('general','VEL_SPEED_MAX',4.0)
        self.vel_speed_last = 0.0   # last recorded speed
        self.vel_accel = balloon_config.config.get_float('general','VEL_ACCEL', 0.5)    # maximum acceleration in m/s/s
        self.vel_dist_ratio = balloon_config.config.get_float('general','VEL_DIST_RATIO', 0.5) 

        # pitch angle to hit balloon at.  negative means come down from above
        self.vel_pitch_target = math.radians(balloon_config.config.get_float('general','VEL_PITCH_TARGET',-5.0))

        # velocity controller update rate
        self.vel_update_rate = balloon_config.config.get_float('general','VEL_UPDATE_RATE_SEC',0.2)

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

            # check if we have a vehicle
            if self.vehicle is None:
                self.vehicle = self.api.get_vehicles()[0]
                return

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
            else:
                self.mission_cmds = None

            # To-Do: if we wish to have the same home position as the flight controller
            # we must download the home waypoint again whenever the vehicle is armed

        # return whether home has been initialised or not
        return self.home_initialised

    # checks if video output should be started
    def check_video_out(self):

        # return immediately if video has already been started
        if not self.writer is None:
            return

        # start video once vehicle is armed
        if self.vehicle.armed:
            self.writer = balloon_video.open_video_writer()

    # check_status - poles vehicle' status to determine if we are in control of vehicle or not
    def check_status(self):

        # we are active in guided mode
        if self.vehicle.mode.name == "GUIDED":
            if not self.controlling_vehicle:
                self.controlling_vehicle = True
                # clear out any limits on balloon position
                self.mission_alt_min = 1
                self.mission_alt_max = 0
                self.mission_distance_max = 50
                # start search for balloon
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

            # ninety two is the MAVLink id for Nav-Guided-Enable commands
            if active_command_id == 92:
                if not self.controlling_vehicle:
                    self.controlling_vehicle = True
                    self.mission_alt_min = self.vehicle.commands[active_command].param2
                    self.mission_alt_max = self.vehicle.commands[active_command].param3
                    self.mission_distance_max = self.vehicle.commands[active_command].param4
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
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0b0000111111000111, # type_mask (only speeds enabled)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used) 
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

    # get_frame - get a single frame from the camera or simulator
    def get_frame(self):
        if self.use_simulator:
            veh_pos = PositionVector.get_from_location(self.vehicle.location)
            frame = balloon_sim.get_simulated_frame(veh_pos, self.vehicle.attitude.roll, self.vehicle.attitude.pitch, self.vehicle.attitude.yaw)
        else:
            frame = balloon_video.get_image()
        return frame

    # get image from balloon_video class and look for balloon, results are held in the following variables:
    #    self.balloon_found : set to True if balloon is found, False otherwise
    #    self.balloon_pitch : earth frame pitch (in radians) from vehicle to balloon (i.e. takes account of vehicle attitude)
    #    self.balloon_heading : earth frame heading (in radians) from vehicle to balloon
    #    self.balloon_distance : distance (in meters) from vehicle to balloon
    #    self.balloon_pos : position vector of balloon's position as an offset from home in meters
    def analyze_image(self):

        # record time
        now = time.time()

        # capture vehicle position
        self.vehicle_pos = PositionVector.get_from_location(self.vehicle.location)

        # capture vehicle attitude in buffer
        self.att_hist.update()

        # get delayed attitude from buffer
        veh_att_delayed = self.att_hist.get_attitude(now - self.attitude_delay)

        # get new image from camera
        f = self.get_frame()

        # look for balloon in image using blob detector        
        self.balloon_found, xpos, ypos, size = balloon_finder.analyse_frame(f)

        # add artificial horizon
        balloon_finder.add_artificial_horizon(f, veh_att_delayed.roll, veh_att_delayed.pitch)

        if self.balloon_found:
            # record time balloon was found
            self.last_spotted_time = now
        
            # convert x, y position to pitch and yaw direction (in radians)
            self.balloon_pitch, self.balloon_heading = balloon_finder.pixels_to_direction(xpos, ypos, veh_att_delayed.roll, veh_att_delayed.pitch, veh_att_delayed.yaw)
            self.balloon_pitch = math.radians(self.balloon_pitch)
            self.balloon_pitch_top = self.balloon_pitch + balloon_video.pixels_to_angle_y(size)  # add balloon radius so we aim for top of balloon
            self.balloon_heading = math.radians(self.balloon_heading)

            # get distance
            self.balloon_distance = get_distance_from_pixels(size, balloon_finder.balloon_radius_expected)

            # updated estimated balloon position
            self.balloon_pos = balloon_finder.project_position(self.vehicle_pos, self.balloon_pitch, self.balloon_heading, self.balloon_distance)

        # save image for debugging later
        if not self.writer is None:
            self.writer.write(f)

    # start_search - start search for balloon
    def start_search(self):
        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle:
            return
        # initialise search variables
        self.search_state = 1       # 0 = not search, 1 = spinning and looking, 2 = rotating to best balloon and double checking, 3 = finalise yaw
        self.search_balloon_pos = None
        self.search_start_heading = self.vehicle.attitude.yaw
        self.search_target_heading = self.search_start_heading
        self.search_total_angle = 0
        self.targeting_start_time = 0
        # reset vehicle speed
        self.vel_speed_last = 0

    # search - spin vehicle looking for balloon
    # analyze_image should have been called just before and should have filled in self.balloon_found, self.balloon_pos, balloon_distance, balloon_pitch and balloon_yaw
    def search_for_balloon(self):

        # exit immediately if we are not controlling the vehicle or do not know our position
        if not self.controlling_vehicle or self.vehicle_pos is None:
            return

        # exit immediately if we are not searching
        if self.search_state <= 0:
            return

        # search states: 0 = not searching, 1 = spinning and looking, 2 = rotating to best balloon and double checking, 3 = finalise yaw

        # search_state = 1: spinning and looking
        if (self.search_state == 1):

            # check if we have seen a balloon
            if self.balloon_found:
                # if this is the first balloon we've found or the closest, store it's position as the closest
                if (self.search_balloon_pos is None) or (self.balloon_distance < self.search_balloon_distance):
                    # check distance is within acceptable limits
                    if (self.mission_alt_min == 0 or self.balloon_pos.z >= self.mission_alt_min) and (self.mission_alt_max == 0 or self.balloon_pos.z <= self.mission_alt_max) and (self.mission_distance_max == 0 or self.balloon_distance <= self.mission_distance_max):
                        # record this balloon as the closest
                        self.search_balloon_pos = self.balloon_pos
                        self.search_balloon_heading = self.balloon_heading
                        self.search_balloon_pitch_top = self.balloon_pitch_top  # we target top of balloon
                        self.search_balloon_distance = self.balloon_distance
                        print "Found Balloon at heading:%f Alt:%f Dist:%f" % (math.degrees(self.balloon_heading), self.balloon_pos.z, self.balloon_distance)
                    else:
                        print "Balloon Ignored at heading:%f Alt:%f Dist:%f" % (math.degrees(self.balloon_heading), self.balloon_pos.z, self.balloon_distance)

            # update yaw so we keep spinning
            if math.fabs(wrap_PI(self.vehicle.attitude.yaw - self.search_target_heading)) < math.radians(self.search_yaw_speed * 2.0):
                # increase yaw target
                self.search_target_heading = self.search_target_heading - math.radians(self.search_yaw_speed)
                self.search_total_angle = self.search_total_angle + math.radians(self.search_yaw_speed)
                # send yaw heading
                self.condition_yaw(math.degrees(self.search_target_heading))

                # end search if we've gone all the way around
                if self.search_total_angle >= math.radians(360):
                    # if we never saw a balloon then just complete (return control to user or mission)
                    if self.search_balloon_pos is None:
                        print "Search did not find balloon, giving up"
                        self.search_state = 0
                        self.complete()
                    else:
                        # update target heading towards closest balloon and send to flight controller
                        self.search_target_heading = self.search_balloon_heading
                        self.condition_yaw(math.degrees(self.search_target_heading))
                        self.search_state = 2   # advance towards rotating to best balloon stage
                        print "best balloon at %f" % math.degrees(self.search_balloon_heading)

        # search_state = 2: rotating to best balloon and double checking
        elif (self.search_state == 2):
            # check yaw is close to target
            if math.fabs(wrap_PI(self.vehicle.attitude.yaw - self.search_target_heading)) < math.radians(5):
                # reset targeting start time
                if self.targeting_start_time == 0:
                    self.targeting_start_time = time.time()

                # if targeting time has elapsed, double check the visible balloon is within acceptable limits
                if (time.time() - self.targeting_start_time > self.targeting_delay_time):
                    if self.balloon_found and (not self.balloon_pos is None) and (self.mission_alt_min == 0 or self.balloon_pos.z >= self.mission_alt_min) and (self.mission_alt_max == 0 or self.balloon_pos.z <= self.mission_alt_max) and (self.mission_distance_max == 0 or self.balloon_distance <= self.mission_distance_max):
                        # balloon is within limits so reset balloon target
                        self.search_balloon_pos = self.balloon_pos
                        self.search_balloon_heading = self.balloon_heading
                        self.search_balloon_pitch_top = self.balloon_pitch_top
                        self.search_balloon_distance = self.balloon_distance
                        # move to final heading
                        self.search_target_heading = self.search_balloon_heading
                        self.condition_yaw(math.degrees(self.search_target_heading))
                        # move to finalise yaw state
                        self.search_state = 3
                        print "Balloon was near expected heading, finalising yaw to %f" % (math.degrees(self.search_balloon_heading))

                    # we somehow haven't found the balloon when we've turned back to find it so giveup
                    elif (time.time() - self.targeting_start_time) > (self.targeting_delay_time + 1.0):
                        print "Balloon was not at expected heading: %f, giving up" % (math.degrees(self.search_target_heading))
                        self.search_state = 0
                        self.complete()

        # search_state = 3: finalising yaw
        elif (self.search_state == 3):
            # check yaw is close to target
            if math.fabs(wrap_PI(self.vehicle.attitude.yaw - self.search_target_heading)) < math.radians(5):
                # complete search, move should take-over
                # Note: we don't check again for the balloon, but surely it's still there
                self.search_state = 0
                print "Finalised yaw to %f, beginning run" % (math.degrees(self.search_balloon_heading))

    # move_to_balloon - velocity controller to drive vehicle to balloon
    #    analyze_image should have been called prior to this and filled in self.balloon_found, balloon_pitch, balloon_heading, balloon_distance 
    def move_to_balloon(self):

        # exit immediately if we are not controlling the vehicle
        if not self.controlling_vehicle:
            return

        # get current time
        now = time.time()

        # exit immediately if it's been too soon since the last update
        if (now - self.guided_last_update) < self.vel_update_rate:
            return;

        # if we have a new balloon position recalculate velocity vector
        if (self.balloon_found):

            # calculate change in yaw since we began the search
            yaw_error = wrap_PI(self.balloon_heading - self.search_balloon_heading)

            # calculate pitch vs ideal pitch angles.  This will cause to attempt to get to 5deg above balloon 
            pitch_error = wrap_PI(self.balloon_pitch_top - self.vel_pitch_target)

            # get time since last time velocity pid controller was run
            dt = self.vel_xy_pid.get_dt(2.0)

            # get speed towards balloon based on balloon distance
            speed = self.balloon_distance * self.vel_dist_ratio

            # apply min and max speed limit
            speed = min(speed, self.vel_speed_max)
            speed = max(speed, self.vel_speed_min)

            # apply acceleration limit
            speed_chg_max = self.vel_accel * dt
            speed = min(speed, self.vel_speed_last + speed_chg_max)
            speed = max(speed, self.vel_speed_last - speed_chg_max)

            # record speed for next iteration
            self.vel_speed_last = speed

            # calculate yaw correction and final yaw movement
            yaw_correction = self.vel_xy_pid.get_pid(yaw_error, dt)
            yaw_final = wrap_PI(self.search_balloon_heading + yaw_correction)

            # calculate pitch correction and final pitch movement
            pitch_correction = self.vel_z_pid.get_pid(pitch_error, dt)
            pitch_final = wrap_PI(self.search_balloon_pitch_top + pitch_correction)
            
            # calculate velocity vector we wish to move in
            self.guided_target_vel = balloon_finder.get_ef_velocity_vector(pitch_final, yaw_final, speed)

            # send velocity vector to flight controller
            self.send_nav_velocity(self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2])
            self.guided_last_update = now

        # if have not seen the balloon
        else:
            # if more than a few seconds has passed without seeing the balloon give up
            if now - self.last_spotted_time > self.lost_sight_timeout:
                if self.debug:
                    print "Lost Balloon, Giving up"
                # To-Do: start searching again or maybe slowdown?
                self.complete()

        # FIXME - check if vehicle altitude is too low
        # FIXME - check if we are too far from the desired flightplan

    def run(self):
        while not self.api.exit:

            # only process images once home has been initialised
            if self.check_home():

                # start video if required
                self.check_video_out()
    
                # check if we are controlling the vehicle
                self.check_status()

                # look for balloon in image
                self.analyze_image()

                # search or move towards balloon
                if self.search_state > 0:
                    # search for balloon
                    self.search_for_balloon()
                else:
                    # move towards balloon
                    self.move_to_balloon()

            # Don't suck up too much CPU, only process a new image occasionally
            time.sleep(0.05)

        if not self.use_simulator:
            balloon_video.stop_background_capture()

    # complete - balloon strategy has somehow completed so return control to the autopilot
    def complete(self):
        # debug
        if self.debug:
            print "Complete!"

        # stop the vehicle and give up control
        if self.controlling_vehicle:
            self.guided_target_vel = (0,0,0)
            self.send_nav_velocity(self.guided_target_vel[0], self.guided_target_vel[1], self.guided_target_vel[2])
            self.guided_last_update = time.time()

        # if in GUIDED mode switch back to LOITER
        if self.vehicle.mode.name == "GUIDED":
            self.vehicle.mode = VehicleMode("LOITER")
            self.vehicle.flush()

        # if in AUTO move to next command
        if self.vehicle.mode.name == "AUTO":
            self.advance_current_cmd();

        # flag we are not in control of the vehicle
        self.controlling_vehicle = False
            
        return

strat = BalloonStrategy()
strat.run()

