# Prepare for Python3 conversion
from __future__ import absolute_import
from __future__ import print_function
import json
import numpy as np
import math

from vehicle.skills.skills import Skill
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.motions.goto_motion import GotoMotion
from vehicle.skills.util.transform import Rot3, Transform

### Globals
global received_gui_speed
received_gui_speed = None


### ComLink skill
class ComLink(Skill):
    """ Communicate with a client device over HTTP.
    This skill displays basic information on the phone, but also
    sends and receives data from an external python script.
    See skydio_client.py
    """

    def __init__(self):
        super(ComLink, self).__init__()
        self.data = {}
        self.pressed = False
        self.motion = None

    def handle_rpc(self, api, message):
        """ A client send custom message to this Skill. """
        global received_gui_speed
        # Serialization format is arbitrary. Here we use assume the client is sending json
        self.data = json.loads(message)

        # Populate a dictionary with response values
        response = {}

        # Vehicle states
        position = api.vehicle.get_position()
        if position is not None:
            response['position'] = position.tolist()
        speed = api.vehicle.get_speed()
        if speed is not None:
            response['speed'] = speed
        quaternion = api.vehicle.get_orientation().quaternion()
        if quaternion is not None:
            response['orientation'] = quaternion.tolist()
        # rpy = api.vehicle.get_orientation().rpy()
        # if rpy is not None:
        #     response['rpy'] = rpy.tolist()

        # Button status
        if self.pressed:
            response['pressed_button'] = True
            # Clear the value now that we've sent it to the client
            self.pressed = False
        
        # Update speed 
        if 'speed' in self.data:
            received_gui_speed = self.data['speed']

        # Move forward X meters
        if 'forward' in self.data:
            # Move to a position in front of the vehicle's main camera.
            nav_T_cam = api.vehicle.get_camera_trans()
            cam_t_point = np.array([self.data['forward'], 0, 0])
            nav_t_point = nav_T_cam * cam_t_point
            nav_T_goal = Transform(nav_T_cam.rotation(), nav_t_point)

            # Get optional speed parameter.
            speed = self.data.get('speed', 2.0)

            # Create Motion object that will manage vehicle position updates over time.
            self.motion = GotoMotion(nav_T_goal, params=dict(speed=speed))

        # Update pose
        if 'pose' in self.data:
            
            ### Creating Transform
            translation = None
            rotation = None

            # Translation
            if sum(self.data['pose'][0]) != 0: # Received translation values
                translation = np.asarray(self.data['pose'][0])
            else:
                translation = api.vehicle.get_position()
            
            # Rotation
            rpy_rad = [self.data['pose'][1][0], self.data['pose'][1][1], self.data['pose'][1][2]]
            if sum(self.data['pose'][1]) != 0: # Received rotation values
                rotation = Rot3.Ypr(rpy_rad[2], rpy_rad[1], rpy_rad[0])
            else:
                rotation = api.vehicle.get_camera_trans().rotation()
            
            print(rotation.rpy())
            nav_T_goal = Transform(rotation, translation)

            # Defining motion
            speed = received_gui_speed if received_gui_speed is not None else 2.0
            self.motion = GotoMotion(nav_T_goal, params=dict(speed=speed))
            
        # Update the layout every time we get a request.
        self.set_needs_layout()

        # Serialization format is arbitrary. Here we send json.
        return json.dumps(response)

    def button_pressed(self, api, button_id):
        """ The user pressed a button in the Skydio app. """
        # Pressing the STOP button should cancel the motion.
        if button_id == 'stop':
            self.motion = None
        elif button_id == 'send':
            # Record for later
            self.pressed = True
        else:
            print('unknown button {}'.format(button_id))

    def get_onscreen_controls(self, api):
        """ Configure the ui. """
        controls = {}
        if self.motion:
            controls['show_stop'] = True
            controls['height_slider_enabled'] = True #DEBUG: Want height slider when com_link is going
        else:
            controls['show_stop'] = False
            controls['height_slider_enabled'] = True

        title = type(self.motion) if self.motion else ''
        controls['title'] = str(self.data.get('title', title))
        controls['detail'] = str(self.data.get('detail', ''))
        if not self.pressed:
            controls['buttons'] = [UiButton('send', label='Ping')]
        else:
            controls['buttons'] = []
        return controls

    def update(self, api):
        """ Control the vehicle. """
        global received_gui_speed
        # If we're executing a motion, disable phone commands and update the motion
        # TODO: what is mapping between input speed and skydio speed? ex input of 1.5 generates speed of ~0.1-0.3

        if self.motion:
            api.phone.disable_movement_commands()
            self.motion.update(api)

            api.planner.settings.obstacle_safety = 1.0
            api.movement.set_max_speed(10.0)

            # When the motion completes, clear it and update the UI.
            if self.motion.done:
                print("motion complete")
                self.motion = None
                self.set_needs_layout()
        else:
            # Otherwise, just listen to the phone.
            api.phone.enable_movement_commands()