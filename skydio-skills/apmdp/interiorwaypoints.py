from __future__ import absolute_import
from __future__ import print_function
import json
import enum
import numpy as np

from vehicle.skills.skills import Skill

from shared.util.time_manager import time_manager as tm
from vehicle.skills.util.motions.motion import Motion

ACCURACY_THRESHOLD = 0.4 # Accepting radius of each waypoint (m)
HOLD_WAYPOINT_THRESHOLD = 0.5 # How long to hold at each waypoint (s)

class MissionStatus(enum.Enum):
    # Awaiting command
    CONFIG_REQUIRED = 0

    # Command received, calibrating GPS
    CALIBRATING = 1

    # Mission is running
    IN_PROGRESS = 2

    # Mission finished succesfully
    COMPLETE = 3

    # Aborted mission
    ABORTED = 4
class LTLWaypoints(Skill):
    def __init__(self):
        super(LTLWaypoints, self).__init__()
        self._status = MissionStatus.CONFIG_REQUIRED

        self._waypoints = []
        self._waypointsIndex = 0

        self._calibrate_start_utime = None
        self._hold_utime = None

        self._motionComplete = False
        self._execute = False
        self._sendComplete = False

        self._counter = 0

    # Create series of waypoints from LTL input
    # INPUT: positions of landmarks (extracted from LTL)
    # OUTPUT: self._waypoints in order populated with flight plan
    def mission_set_waypoints(self, api, data):
        assert api.waypoints.ready_for_waypoints(), 'waypoint api not ready'
        for counter, point in enumerate(data):
            try:
                #self._waypoints.append(api.waypoints.save_nav_location(np.array(point), orientation=None, waypoint_id=counter))
                self._waypoints.append(api.waypoints.save_gps_location(point[0], point[1], nav_z=4, waypoint_id=counter))
                #self._waypoints.append(api.waypoints.save_gps_location(point[0] - 4.3543, point[1] - 50.832, nav_z=15, waypoint_id=counter))
            except:
                print("Caught GPS waypoint error")
                self._status = MissionStatus.ABORTED
    # Determine if R1 is ready to start mission
    # Input: API
    # Output: MissionStatus set to in progress once GPS is calibrated
    def mission_calibration(self, api):
        # Wait for GPS initialization
        ready = api.waypoints.ready_for_waypoints()
        if not ready:
            if self._calibrate_start_utime is None:
                self._calibrate_start_utime = api.utime
            
            # Abort if calibration takes longer than 10 seconds
            elapsed = tm.utime_to_seconds(api.utime - self._calibrate_start_utime)
            print("Calibrating...")
            if elapsed < 15.0:
                # Move R1, trying to gain GPS
                api.movement.set_desired_vel_body(np.array([1.0, 0.0, 0.0])) 
        elif ready:
            print("Calibrated!")
            self.mission_set_waypoints(api, self._received_data)
            self._status = MissionStatus.IN_PROGRESS # Calibration completed, mission can begin

    # Main loop. This updates at 8hz
    def update(self, api):
        # No command recieved, do nothing
        if self._status == MissionStatus.CONFIG_REQUIRED:
            '''
            ###MANUAL WAYPOINT OVERRIDE###
            gps_coord_1 = (37.4719764, -122.2338829)
            gps_coord_2 = (37.4719765, -122.2325429)
            gps_coord_3 = (37.4729765, -122.2334429)
            # nav_coord_1 = api.waypoints.gps_to_nav(gps_coord_1[0], gps_coord_1[1])
            # nav_coord_2 = api.waypoints.gps_to_nav(gps_coord_2[0], gps_coord_2[1])
            # nav_coord_3 = api.waypoints.gps_to_nav(gps_coord_3[0], gps_coord_3[1])
            waypoint_1 = api.waypoints.save_gps_location(gps_coord_1[0], gps_coord_1[1], waypoint_id=1)
            waypoint_2 = api.waypoints.save_gps_location(gps_coord_2[0], gps_coord_2[1], waypoint_id=2)
            waypoint_3 = api.waypoints.save_gps_location(gps_coord_3[0], gps_coord_3[1], waypoint_id=3)
            self._waypoints = [waypoint_1, waypoint_2, waypoint_3]
            self._status = MissionStatus.IN_PROGRESS
            ###############################
            '''
            return 
        # Continue calibration until R1 is ready to begin mission
        if self._status == MissionStatus.CALIBRATING:
            self.mission_calibration(api)
        # Execute flight path
        if self._status == MissionStatus.IN_PROGRESS:
            if not self._execute: # Print the flight plan before starting
                print("Flight plan (nav):")
                print([api.waypoints.get_waypoint_in_nav(p) for p in self._waypoints])
                print("Flight plan (GPS):")
                print([api.waypoints.get_waypoint_in_gps(p) for p in self._waypoints])
                self._execute = True
            api.waypoints.fly_to_waypoint(self._waypoints[self._waypointsIndex])
            print("R1 position:")
            print(api.vehicle.get_position())
            print("Distance to waypoint:")
            print(api.waypoints.distance_to_waypoint(self._waypoints[self._waypointsIndex])) 
            if api.waypoints.distance_to_waypoint(self._waypoints[self._waypointsIndex]) < ACCURACY_THRESHOLD or self._motionComplete:
                if not self._motionComplete:
                    self._motionComplete = True
                if self._waypointsIndex + 1 < len(self._waypoints):
                    if self._hold_utime is None:
                        self._hold_utime = api.utime
                    elapsed = tm.utime_to_seconds(api.utime - self._hold_utime)
                    if elapsed < HOLD_WAYPOINT_THRESHOLD: # Hold drone at waypoint for threshold length
                        print("Holding at waypoint for {} more seconds".format(HOLD_WAYPOINT_THRESHOLD - elapsed))
                        api.movement.set_desired_vel_body(np.array([0.0, 0.0, 0.0])) # Stop drone
                    else:
                        self._hold_utime = None
                        self._motionComplete = False
                        self._waypointsIndex += 1
                        self._counter = 0
                        print("Going to next waypoint")
                else:
                    self._status = MissionStatus.COMPLETE
        if self._status == MissionStatus.COMPLETE:
            if self._execute:
                print("Mission complete")
                self._execute = False
                self._sendComplete = True
            api.movement.set_desired_vel_body(np.array([0.0, 0.0, 0.0])) # Stop drone
        if self._status == MissionStatus.ABORTED:
            if self._execute:
                print("ABORTED, CODE {}. STOPPING R1".format(self._status))
                self._execute = False
            api.movement.set_desired_vel_body(np.array([0.0, 0.0, 0.0])) # Stop drone
        self.set_needs_layout()

        
    # Handles incoming commands
    # Input: Messages sent by Pilot to R1
    # Output: Stores speech to location command sequence and transitions R1 to calibration
    def handle_rpc(self, api, message):
        data = json.loads(message) # Assume json encoding
        if 'abort' in data:
            self._status = MissionStatus.ABORTED # User abort received, land R1
            return 
        if 'move' in data:
            self._received_data = data['move']
            self._status = MissionStatus.CALIBRATING # Command received, moving to calibration

        resp = {}      
        if self._sendComplete:
            resp['status'] = "complete"
            self._sendComplete = False
        if resp:
            return json.dumps(resp)
    
    def button_pressed(self, api, button_id_pressed):
        """
        Handle UI button press events sent from the Skydio app.
        """
        if button_id_pressed == 'stop':
            self._status = MissionStatus.ABORTED

    def get_onscreen_controls(self, api):
        """
        Populate the ui elements that should appear on the phone based on the mission status code.
        """

        buttons = []
        show_stop = True
        controls_enabled = True
        promoted_control_id = None

        if self._status == MissionStatus.CONFIG_REQUIRED:
            title = 'CONFIG_REQUIRED'
            detail = 'Waiting for LTL command'

        elif self._status == MissionStatus.CALIBRATING:
            title = 'CALIBRATING'
            detail = 'GPS Calibration'

        elif self._status == MissionStatus.IN_PROGRESS:
            title = 'IN_PROGRESS'
            detail = "Going to waypoint {}/{}".format(self._waypointsIndex,
                                             len(self._waypoints))

        elif self._status == MissionStatus.COMPLETE:
            title = 'COMPLETE'
            detail = 'LTL command executed'

        elif self._status == MissionStatus.ABORTED:
            title = 'ABORTED'
            detail = 'R1 aborted mission'


        controls = dict(
            title=title,
            detail=detail,
            show_stop=show_stop,
            height_slider_enabled=controls_enabled,
            arrows_enabled=controls_enabled,
            buttons=buttons,
            promoted_control=promoted_control_id,
        )

        return controls
