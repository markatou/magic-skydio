from vehicle.skills.skills import Skill
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.ui import UiSlider
from shared.util.time_manager import time_manager as tm

import numpy as np
import enum

class Status(enum.Enum):
    WAITING = 0
    IN_PROGRESS = 1
    COMPLETE = 2
    ERROR = 3

class DoorFly(Skill):
    def __init__(self):
        super(DoorFly, self).__init__()
        self.first = True
        self.speed = 4.0 # m/s
        self.doorWaypoint = -1 # No id
        self.sentDoorCommand = False
        self.state = Status.WAITING
        self.publish_downsampler = tm.DownSampler(0.5) # for UI updates


    def setDoorWaypoint(self, api):
        self.doorWaypoint = api.waypoints.save_nav_location(api.vehicle.get_position() + np.array([3.0, 0.0, 5.0]), orientation=None, waypoint_id=0)
    
    def get_onscreen_controls(self, api):
        title = 'Waiting for activation'
        detail_text = ''
        buttons = []
        show_stop = True
        controls_enabled = True 
        targets_enabled = False 
        show_slider = False 
        promoted_control_id = ''
        battery_low_land = api.health_monitor.is_battery_critically_low()

        if battery_low_land:
            title = 'Low battery!'
            detail_text = 'Land drone in safe location'
            controls_enabled = True 
        
        elif self.state == Status.WAITING:
            title = 'Press "GO" to start'
            detail_text = 'Drone to waypoint'
            show_stop = False 
            buttons.append(UiButton(identifier='go', label='GO', style='PRIMARY'))
            promoted_control_id = 'go'
        
        elif self.state == Status.IN_PROGRESS:
            title = 'Flight in progress'
        
        elif self.state == Status.COMPLETE:
            title = 'Flight complete, select new skill'
            show_stop = False 
        
        return dict(
            title=title,
            detail=detail_text,
            arrows_enabled=controls_enabled,
            height_slider_enabled=controls_enabled,
            zoom_slider_enabled=controls_enabled,
            steering_enabled=controls_enabled,
            tap_targets_enabled=targets_enabled,
            double_tap_enabled=controls_enabled,
            drag_enabled=controls_enabled,
            show_stop=show_stop,
            promoted_control=promoted_control_id,
            buttons=buttons,
        )
    
    def button_pressed(self, api, button_id_pressed):
        if button_id_pressed == 'go':
            self.state = Status.IN_PROGRESS
        if button_id_pressed == 'stop':
            self.state = Status.COMPLETE
    
    def update(self, api):
        if self.first:
            self.setDoorWaypoint(api)
            self.first = False
        
        if api.vehicle.get_pose():
            if self.state == Status.IN_PROGRESS:
                print("current drone pose:")
                print(api.vehicle.get_position())
                print("going to waypoint#:")
                print(self.doorWaypoint)
                print("waypoint nav info:")
                print(api.waypoints.get_waypoint_in_nav(self.doorWaypoint))
                print("waypoint in gps coords:")
                print(api.waypoints.get_waypoint_in_gps(self.doorWaypoint))
                print("waypoint safety:")
                print(api.waypoints.is_waypoint_safe(api.waypoints.get_waypoint_in_nav(self.doorWaypoint)))
                api.waypoints.fly_to_waypoint(self.doorWaypoint)
            
            if api.waypoints.distance_to_waypoint(self.doorWaypoint) < 0.1: # Assuming the waypoint has been reached
                self.state = Status.COMPLETE
        
        if self.publish_downsampler.ready(api.utime): # Keep UI updated
            self.set_needs_layout() # calls get_onscreen_controls




