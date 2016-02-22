"""
AttitudeHistory class : provides delayed attitude and location information
"""

import math
import time
from pymavlink import mavutil
from dronekit import VehicleMode, Attitude
from balloon_utils import wrap_PI

class AttitudeHistory(object):

    def __init__(self, dronekit_vehicle, max_delay):
        self.vehicle = dronekit_vehicle # reference to drone api's vehicle for retrieving attitude and location
        self.max_delay = max_delay      # maximum expected delay in seconds
        self.last_update = 0            # system time of last update call
        self.att_dict = dict()          # initialise attitude dictionary

        self.max_update_rate = 0.02     # do not store updates at more than 50hz
        # debug testing counter
        self.testing_counter = 0

    # __str__ - print contents of dictionary
    def __str__(self):
        return "AttHist MaxDelay:%d \nAttDict:%s" % (self.max_delay, self.att_dict)

    # update - captures an attitude from the drone api and stores in a dictionary
    def update(self):
        # get current time
        now = time.time()

        # don't update if it's too soon since the last update
        time_diff = now - self.last_update
        if time_diff < self.max_update_rate:
            return

        # exit immediately if vehicle is not initialised
        if self.vehicle is None:
            return

        # check attitude is initialised
        if not self.vehicle.attitude is None:
            # add attitude to dictionary
            self.att_dict[now] = self.vehicle.attitude
            # debug
            #print "Add Att: t:%f r:%f p:%f y:%f" % (now, self.vehicle.attitude.roll, self.vehicle.attitude.pitch, self.vehicle.attitude.yaw) 

        # clear out any old entries
        for t in self.att_dict.keys():
            if t < now - self.max_delay:
                self.att_dict.pop(t)

        # store last update time       
        self.last_update = now

    # get_attitude - get attitude at given time 
    def get_attitude(self, desired_time_in_sec):
        # return current attitude immediately if dictionary is empty
        if len(self.att_dict) == 0:
            return self.vehicle.attitude

        # get times from dict
        keys = sorted(self.att_dict.keys())

        # debug
        #print "AttDict looking for %f" % desired_time_in_sec

        # initialise best before and after keys
        key_before = keys[0]
        time_diff_before = (key_before - desired_time_in_sec)
        key_after = keys[len(keys)-1]
        time_diff_after = (key_after - desired_time_in_sec)

        # handle case where we hit the time exactly or the time is beyond the end
        if (time_diff_before >= 0):
            #debug
            #print "Time %f was before first entry's time %f" % (desired_time_in_sec, key_before)
            return self.att_dict[key_before]
        if (time_diff_after <= 0):
            #debug
            #print "Time %f was after last entry's time %f" % (desired_time_in_sec, key_after)
            return self.att_dict[key_after]

        # iteration through attitude dictionary
        for t in keys:
            # calc this dictionary entry's time diff from the desired time
            time_diff = t - desired_time_in_sec

            # if before the desired time but after the current best 'before' time use it
            if time_diff <= 0 and time_diff > time_diff_before:
                time_diff_before = time_diff
                key_before = t

            # if after the desired time but before the current best 'before' time use it
            if time_diff >= 0 and time_diff < time_diff_after:
                time_diff_after = time_diff
                key_after = t

        # calc time between before and after attitudes
        tot_time_diff = -time_diff_before + time_diff_after

        # debug
        if (tot_time_diff <= 0):
            print "Div By Zero!"
            print "des time:%f" % desired_time_in_sec
            print "num keys:%d" % len(keys)
            print "key bef:%f aft:%f" % (key_before, key_after)
            print "keys: %s" % keys

        # get attitude before and after
        att_before = self.att_dict[key_before]
        att_after = self.att_dict[key_after]

        # interpolate roll, pitch and yaw values
        interp_val = (-time_diff_before / tot_time_diff)
        roll = wrap_PI(att_before.roll + (wrap_PI(att_after.roll - att_before.roll) * interp_val))
        pitch = att_before.pitch + (att_after.pitch - att_before.pitch) * interp_val
        yaw = wrap_PI(att_before.yaw + (wrap_PI(att_after.yaw - att_before.yaw) * interp_val))

        ret_att = Attitude(pitch, yaw, roll)

        return ret_att

    # main - test the class
    def main(self):

        # print object
        print "Test AttitudeHistory"

        for i in range(0,40):
            # add some entries
            self.update()
            time.sleep(0.1)

        # print out dictionaries
        print str(self)

        # retrieve attitude 0.25 sec before
        att = self.get_attitude(time.time()-0.2)
        print "Att 0.25 sec ago: %s" % att

        # test entry which is later than last item
        #att = self.get_attitude(time.time())
        #print "Att Now: %s" % att

        # test entry which is earlier than earliest item
        #att = self.get_attitude(time.time()-10)
        #print "Att 10 sec ago: %s" % att

        # retrieve attitude from last time stored
        #keys = sorted(self.att_dict.keys()) 
        #att = self.get_attitude(keys[len(keys)-1])
        #print "Got attitude: %s" % att

        # retrieve attitude from earliest time stored
        #keys = sorted(self.att_dict.keys()) 
        #att = self.get_attitude(keys[0])
        #print "Got attitude: %s" % att
        
        # test wrap

        # test items are cleared when they get too old

'''
# run the main routine if this is file is called from the command line
if __name__ == "__main__":

    # for testing from the command line
    print "No command line tests supported"

else:
    # for testing in the simulator

    connection_str = balloon_config.config.get_string('dronekit','connection_string','/dev/ttyUSB0') 
    connection_baud = balloon_config.config.get_integer('dronekit','baud',921600)
    vehicle = dronekit.connect(connection_str, connection_baud)

    # create test attitude history class with maximum 2 second delay
    test_atthist = AttitudeHistory(vehicle, 2.0)

    # run tests
    test_atthist.main()
'''