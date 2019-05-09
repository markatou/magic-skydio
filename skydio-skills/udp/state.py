from __future__ import absolute_import
from __future__ import print_function
import json
import numpy as np

from vehicle.skills.skills import Skill


class StateEstimate(Skill):
    """ Control the vehicle from an separate computer via WiFi or USB ethernet. """

    def __init__(self):
        super(StateEstimate, self).__init__()

    def update(self, api):
        # Don't allow subject tracking in this mode.
        api.subject.cancel_subject_tracking(api.utime)

        # Don't allow phone movements.
        api.phone.disable_movement_commands()

        # Set the upper-limit for vehicle speed
        api.movement.set_max_speed(12.0)

        # Publish a status message to the phone with some data in it.
        status = {}
        status['speed'] = api.vehicle.get_speed()
        status['position'] = list(api.vehicle.get_position())
        status['orientation'] = list(api.vehicle.get_orientation().quaternion())
        api.custom_comms.publish_status(json.dumps(status))

        return json.dumps(status)

