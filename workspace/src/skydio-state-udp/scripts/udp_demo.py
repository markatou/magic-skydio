# prep for python 3.0
from __future__ import absolute_import
from __future__ import print_function
import argparse
import os
import threading
import time
import yaml


from http_client import HTTPClient
from udp_link import UDPLink

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose

def main():
    parser = argparse.ArgumentParser(
        description="Control R1 from a computer with a connected gamepad.")
    parser.add_argument('--baseurl', metavar='URL', default='http://192.168.10.1',
                        help='the url of the vehicle')

    parser.add_argument('--skill-key', required=True, type=str,
                        help='name of the RemoteControl skill to run on the vehicle. '
                        'e.g. my_skillset.remote_control.RemoteControl')

    # NOTE: you'll need a token file in order to connect to a simulator.
    # Tokens are NOT required for real R1s.
    parser.add_argument('--token-file',
                        help='path to the auth token for your simulator')

    parser.add_argument('--update-skillsets-email', type=str,
                        help='The email of the user to get skillsets for and send them to the '
                             'vehicle (must be pilot)')
    parser.add_argument('--skydio-api-url', type=str, help='Override the skydio api url')

    parser.add_argument('--takeoff', action='store_true',
                        help='send a takeoff command (must be pilot)')

    parser.add_argument('--land', action='store_true',
                        help='send a land command (must be pilot)')

    args = parser.parse_args()

    # Create the client to use for all requests.
    client = HTTPClient(args.baseurl,
                        pilot=True,
                        token_file=args.token_file)

    if not client.check_min_api_version():
        print('Your vehicle is running an older api version.'
              ' Update recommended in order to enable streaming.')

    if args.update_skillsets_email:
        client.update_skillsets(args.update_skillsets_email,
                                api_url=args.skydio_api_url)

    if args.takeoff:
        # Ensure that the vehicle has taken off before continuing.
        client.takeoff()

    if args.land:
        client.land()
        # Dont do anything else after landing.
        return

    # Periodically poll the status endpoint to keep ourselves the active pilot.
    def update_loop():
        while True:
            client.update_pilot_status()
            time.sleep(2)
    status_thread = threading.Thread(target=update_loop)
    status_thread.setDaemon(True)
    status_thread.start()

    # Create a low-latency udp link for quickly sending messages to the vehicle.
    remote_address = client.get_udp_link_address()
    link = UDPLink(client.client_id, local_port=50112, remote_address=remote_address)

    # Connect the UDPLink to the vehicle before trying to takeoff.
    link.connect()

    # Set R1 to the desired skill
    client.set_skill(args.skill_key)

    # ROS publishers
    pose_pub = rospy.Publisher('pose', Pose, queue_size=10)
    speed_pub = rospy.Publisher('speed', Float32, queue_size=1)

    # Initializing node
    rospy.init_node('skydio_talker_udp', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Continously send requests to the com skill
        link.send_json(args.skill_key, "request")
        msg = link.read()

        if msg: #msg may not always be received (UDP), so we include this check
            stream = yaml.load(msg.data)
            position = stream['position']
            speed = stream['speed']
            orientation = stream['orientation']

            posemsg = Pose()
            posemsg.position.x = position[0]
            posemsg.position.y = position[1]
            posemsg.position.z = position[2]
            posemsg.orientation.x = orientation[0]
            posemsg.orientation.y = orientation[1]
            posemsg.orientation.z = orientation[2]
            posemsg.orientation.w = orientation[3]

            speedmsg = Float32()
            speedmsg.data = speed

            pose_pub.publish(posemsg)
            speed_pub.publish(speedmsg)

        rate.sleep()




        
if __name__ == '__main__':
    main()