"""
Com Link Demo

While flying a vehicle with your phone, this script will send and receive data from a separate computer via HTTP connected over wifi.

Steps:
    1. Connect to R1 with your computer over wifi (or download a token file to use a simulator)
    2. Using your phone, fly the vehicle and switch to the ComLink skill
    3. Run the script, and get messages back from the ComLink skill.

This demo will change the messages that are displayed on the pilot's phone.

You can also use --forward X to send a command that moves the vehicle forward X meters.

Use --loop to see messages repeatedly.
"""
from __future__ import absolute_import
from __future__ import print_function
import argparse
import json
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from http_client import HTTPClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseurl', metavar='URL', default='http://192.168.10.1',
                        help='the url of the vehicle')

    parser.add_argument('--skill-key', type=str,
                        help='the import path of the ComLink Skill already on the R1')

    # NOTE: you'll need a token file in order to connect to a simulator.
    # Tokens are NOT required for real R1s.
    parser.add_argument('--token-file',
                        help='path to the auth token for your simulator')

    # Example actions for the ComLink skill
    parser.add_argument('--forward', metavar='X', type=float,
                        help='move forward X meters.')

    parser.add_argument('--loop', action='store_true',
                        help='keep sending messages')

    # Experimental: save a 720P image from the vehicle as a .png file
    parser.add_argument('--image', action='store_true',
                        help='save an image')

    parser.add_argument('--title', default='Hello World')

    args = parser.parse_args()

    # Create the client to use for all requests.
    client = HTTPClient(args.baseurl,
                        pilot=False,
                        token_file=args.token_file)

    # Create the request that we will send to the ComLink skill.
    request = {
        'title': 'Hello World',
        'detail': 0,
    }
    if args.forward:
        request['forward'] = args.forward

    pose_pub = rospy.Publisher('pose', Pose, queue_size=10)
    speed_pub = rospy.Publisher('speed', Float32, queue_size=1)
    rospy.init_node('skydio_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Continuously poll
    start_time = time.time()
    while not rospy.is_shutdown():
        elapsed_time = int(time.time() - start_time)
        request['detail'] = elapsed_time

        # transport_client output, arbitrary data format. Using JSON here.
        t = time.time()
        response = client.send_custom_comms_receive_parsed(args.skill_key, json.dumps(request)) #comes in the form [[x, y, z], speed]
        dt = int((time.time() - t) * 1000)
        resp = 'JSON response (took {}ms) {}\n'.format(dt, json.dumps(response, sort_keys=True, indent=True))
        print(resp)

        # Publish data on ROS topics
        posemsg = Pose()
        posemsg.position.x = response[0][0]
        posemsg.position.y = response[0][1]
        posemsg.position.z = response[0][2]

        speedmsg = Float32()
        speedmsg.data = response[1]

        print("--ROSLOG--")
        rospy.loginfo(posemsg)
        rospy.loginfo(speedmsg)

        pose_pub.publish(posemsg)
        speed_pub.publish(speedmsg)
        rate.sleep()

        if args.image:
            print('Requesting image')
            client.save_image(filename='image_{}.png'.format(elapsed_time))

        if args.loop:
            time.sleep(1.0)

            # Don't repeat the forward command.
            if 'forward' in request:
                del request['forward']

        else:
            # Exit the loop.
            break


if __name__ == '__main__':
    main()
