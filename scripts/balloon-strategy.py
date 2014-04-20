import time
from droneapi.lib import VehicleMode, Location
from find_balloon import get_camera, open_video_writer

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

        # The module only prints log messages unless the vehicle is in GUIDED mode (for testing).
        # Once this seems to work reasonablly well change self.debug to False and then it will
        # actually _enter_ guided mode when it thinks it sees a balloon
        self.debug = True

        self.min_wpt = 1 # If the vehicle is in its AUTO mission, we only look for balloons between these two wpts
        self.max_wpt = 4

        self.camera = get_camera()
        self.writer = open_video_writer()

    def get_frame(self):
        _, frame = self.camera.read()
        return frame

    def analyze_image(self):
        f = self.get_frame()
        self.writer.write(f) # For debugging later...
        print "FIXME - add image analysis and fancy-pants math"

        # FIXME - analyze the image to get a score indicating likelyhood there is a balloon and if it
        # is there the x & y position in frame of the largest balloon
        # FIXME - check to see if the image looks like a balloon
        # FIXME - check if the balloon gets larger if we think we are approaching it

        found_in_image = True # replace this with real code
        xpos = 200
        ypos = 300

        if found_in_image:
            vehicle_pos = self.vehicle.location

            # FIXME - do math based on current vehicle loc and the x,y frame position
            # (Someone needs a brain that still remembers trig)
            lat_offset = 0.1 # FIXME
            lon_offset = 0.1
            alt_offset = 1
            target_pos = Location(vehicle_pos.lat + lat_offset, vehicle_pos.lon + lon_offset, vehicle_pos.alt + alt_offset, vehicle_pos.is_relative)

            # FIXME - check if vehicle altitude is too low
            # FIXME - check if we are too far from the desired flightplan

            self.balloon_loc = target_pos



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
            time.sleep(0.5)
        self.camera.release()

strat = BalloonStrategy()
strat.run()

