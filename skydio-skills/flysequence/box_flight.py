import enum
import math
import numpy as np

from shared.util.error_reporter import error_reporter as er
from shared.util.time_manager import time_manager as tm
from vehicle.skills.skills import Skill
from vehicle.skills.util import core
from vehicle.skills.util.motions import CableMotion
from vehicle.skills.util.motions import LookatMotion
from vehicle.skills.util.motions import OrbitMotion
from vehicle.skills.util.transform import Rot3
from vehicle.skills.util.transform import Transform
from vehicle.skills.util.ui import UiButton
from vehicle.skills.util.ui import UiSlider

class Status(enum.Enum):
    AUTH_WAIT = 0
    IN_PROGRESS = 1
    COMPLETE = 2
    ERROR = 3

class MakeBox(Skill):

    USER_SETTINGS = (
        UiSlider(
            identifier='box_size',
            label='Size',
            detail='Edge length of box R1 should fly',
            value=5.0,
            min_value=1.0,
            max_value=10.0,
            units='m',
        ),
        UiSlider(
            identifier='speed',
            label='Speed',
            detail='Speed of R1',
            value=5.0,
            min_value=1.0,
            max_value=15.0,
            units='m/s'
        ),
    )

    def __init__(self):
        super(MakeBox, self).__init__() #gives access to functions in superclasses
        self.state = Status.AUTH_WAIT
        self.has_subject = False
        self.publish_downsampler = tm.DownSampler(0.5) #for UI updates

        self.params = core.AttrDict()
        self.params.speed = 6.0
        self.params.min_turn_rate = 1.0  #[rad/update]
        self.params.max_turn_rate = 15.0 #[rad/update]
        self.first = True
        self.motions = []
        self.motion_index = None

    def button_pressed(self, api, button_pressed):
        if button_pressed == "go_button":
            self.state = Status.IN_PROGRESS

            ###Get current location###
            curr_cam_location = api.vehicle.get_camera_trans()

            yaw = curr_cam_location.get_euler_angles()[2] #tuple of [roll, pitch, yaw]
            curr_location_flat = Transform(
                rotation = Rot3.Ypr(yaw, 0, 0),
                translation = curr_cam_location.translation()
            )

            ###Get flight path information###
            size = 3.0

            ###Flight path###
            start_location = curr_location_flat.copy()

            nav_edge_1 = Transform(
                rotation = curr_location_flat.rotation(),
                translation = curr_location_flat * np.array([0, size / 2, 0])
            )

            nav_edge_2 = Transform(
                rotation = nav_edge_1.rotation(),
                translation = nav_edge_1 * np.array([-(size / 2), 0, 0])
            )

            nav_edge_3 = Transform(
                rotation = nav_edge_2.rotation(),
                translation = nav_edge_2 * np.array([0, -(size / 2), 0])
            )

            nav_edge_4 = Transform(
                rotation = nav_edge_3.rotation(),
                translation = nav_edge_3 * np.array([size / 2, 0, 0])
            )

            # Storing flight-path information
            self.motions = [
                CableMotion(start_location, nav_edge_1, self.params),
                CableMotion(nav_edge_1, nav_edge_2, self.params),
                CableMotion(nav_edge_2, nav_edge_3, self.params),
                CableMotion(nav_edge_3, nav_edge_4, self.params)
            ]
            self.motion_index = 0

        if button_pressed == "skip":
            self.motion_index += 1
            if self.motion_index >= len(self.motions):
                self.state = Status.COMPLETE

        if button_pressed == "stop":
            self.state = Status.COMPLETE

    def get_next_motion(self):
        if self.motion_index is None or self.motion_index >= len(self.motions):
            return None
        return self.motions[self.motion_index]

    def update(self, api):
        if self.first:
            self.box(api)
            self.first = False
        ###Execute next motion###
        if api.vehicle.get_pose():
            if self.state == Status.IN_PROGRESS:
                motion = self.get_next_motion()
                if motion:
                    motion.update(api) #Executing the motion
                    if motion.done:
                        motion.reset(api) #Resets state of motion execution
                        self.motion_index += 1
                else:
                    er.REPORT_STATUS("No more flight path data available")
                    self.state = Status.COMPLETE
        ###Keep the UI updated###
        if self.publish_downsampler.ready(api.utime):
            self.set_needs_layout() #indicates UI needs to be changed, calls get_onscreen_controls

    def get_onscreen_controls(self, api): #needs to be named this per API spec
        ###Default UI values###
        title = ''
        detail_text = ''
        buttons = []
        show_stop = True
        controls_enabled = False
        targets_enabled = False
        show_slider = False
        promoted_control_id = ''
        battery_low_land = api.health_monitor.is_battery_critically_low()

        ###UI configurations###
        #Low battery
        if battery_low_land:
            title = 'Battery is low'
            detail_text = 'Drone needs to land, low battery'
            controls_enabled = True

        #Setup
        elif self.state == Status.AUTH_WAIT:
            title = 'Press Go to begin'
            detail_text = 'Drone will follow box flight path'
            controls_enabled = True
            show_slider = True #Show size slider
            show_stop = False #Show stop button
            targets_enabled = False #Show tappable targets (for follow functionality)
            buttons.append(UiButton(identifier='go_button', label='Go', style='PRIMARY'))
            promoted_control_id = 'go_button' #promoted over video feed

        #In-Flight
        elif self.state == Status.IN_PROGRESS:
            title = 'Flight in progress'
            detail_text = 'Executing movement {}'.format(self.motion_index)
            targets_enabled = False
            buttons.append(UiButton(identifier='skip_button', label='skip', style='PRIMARY'))
            promoted_control_id = 'speed'

        #Flight completed
        elif self.state == Status.COMPLETE:
            title = 'Press Go to start another flight'
            controls_enabled = True
            show_slider = True
            show_stop = False
            targets_enabled = False
            buttons.append(UiButton(identifier='go_button', label='Go', style='PRIMARY'))
            promoted_control_id = 'go_button' #promoted over video feed

        ###Return UI in dict###
        return dict(
            title=title,
            detail=detail_text,
            arrows_enabled=controls_enabled,
            height_slider_enabled=controls_enabled,
            zoom_slider_enabled=controls_enabled and self.has_subject,
            steering_enabled=controls_enabled and not self.has_subject,
            tap_targets_enabled=targets_enabled,
            double_tap_enabled=controls_enabled,
            drag_enabled=controls_enabled,
            show_stop=show_stop,
            promoted_control='box_size' if show_slider else promoted_control_id,
            buttons=buttons,
        )


    def box(self, api): 
        self.state = Status.IN_PROGRESS

        ###Get current location###
        curr_cam_location = api.vehicle.get_camera_trans()

        yaw = curr_cam_location.get_euler_angles()[2] #tuple of [roll, pitch, yaw]
        curr_location_flat = Transform(
                rotation = Rot3.Ypr(yaw, 0, 0),
                translation = curr_cam_location.translation()
        )

        ###Get flight path information###
        size = 4.0

        ###Flight path###
        start_location = curr_location_flat.copy()

        nav_edge_1 = Transform(
                rotation = curr_location_flat.rotation(),
                translation = curr_location_flat * np.array([0, 0, size / 2])
        )

        nav_edge_2 = Transform(
            rotation = nav_edge_1.rotation(),
            translation = nav_edge_1 * np.array([0, -(size / 2), 0])
        )

        nav_edge_3 = Transform(
            rotation = nav_edge_2.rotation(),
            translation = nav_edge_2 * np.array([0, 0, -(size / 2)])
        )

        nav_edge_4 = Transform(
            rotation = nav_edge_3.rotation(),
            translation = nav_edge_3 * np.array([0, size / 2, 0])
        )

        # Storing flight-path information
        self.motions = [
            CableMotion(start_location, nav_edge_1, self.params),
            CableMotion(nav_edge_1, nav_edge_2, self.params),
            CableMotion(nav_edge_2, nav_edge_3, self.params),
            CableMotion(nav_edge_3, nav_edge_4, self.params)
        ]
        self.motion_index = 0
