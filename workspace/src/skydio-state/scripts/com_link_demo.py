from __future__ import absolute_import
from __future__ import print_function
import argparse
import json
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from http_client import HTTPClient

### Globals
global moveForwardVal 
moveForwardVal = 0

global speedVal
speedVal = 0

global pose
pose = None

### Callbacks
def ml_callback(msg):
    global moveForwardVal
    moveForwardVal = msg.data

def speed_callback(msg):
    global speedVal
    speedVal = msg.data

def pose_callback(msg):
    global pose 
    pose = [[msg.position.x, msg.position.y, msg.position.z],
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]]

### Run
def main():
    global moveForwardVal
    global speedVal
    global pose

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseurl', metavar='URL', default='http://192.168.10.1',
                        help='the url of the vehicle')

    parser.add_argument('--skill-key', type=str,
                        help='the import path of the ComLink Skill already on the R1')

    # NOTE: you'll need a token file in order to connect to a simulator.
    # Tokens are NOT required for real R1s.
    parser.add_argument('--token-file',
                        help='path to the auth token for your simulator')

    parser.add_argument('--loop', action='store_true',
                        help='keep sending messages')

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

    # ROS publishers
    pose_pub = rospy.Publisher('pose', Pose, queue_size=10)
    speed_pub = rospy.Publisher('speed', Float32, queue_size=1)

    # Initializing node
    rospy.init_node('skydio_talker', anonymous=True)

    # Listen to movement publishers (for control support)
    rospy.Subscriber('go_forward', Int64, ml_callback)
    rospy.Subscriber('gui_speed_update', Float32, speed_callback)
    rospy.Subscriber('gui_pose_update', Pose, pose_callback)
    
        
    rate = rospy.Rate(10) # 10hz

    # Continuously poll
    start_time = time.time()
    while not rospy.is_shutdown():
        if moveForwardVal != 0:
            request['forward'] = moveForwardVal
        
        if speedVal != 0:
            request['speed'] = speedVal
        
        if pose:
            request['pose'] = pose 

        elapsed_time = int(time.time() - start_time)
        request['detail'] = elapsed_time

        # transport_client output, arbitrary data format. Using JSON here.
        t = time.time()

        # Response comes in form [[position], [orientation (quaternion)], speed]
        response = client.send_custom_comms_receive_parsed(args.skill_key, json.dumps(request)) 
        
        dt = int((time.time() - t) * 1000)
        resp = 'JSON response (took {}ms) {}\n'.format(dt, json.dumps(response, sort_keys=True, indent=True))
        print(resp)

        # Publish data on ROS topics
        posemsg = Pose()
        posemsg.position.x = response[0][0]
        posemsg.position.y = response[0][1]
        posemsg.position.z = response[0][2]
        posemsg.orientation.x = response[1][0]
        posemsg.orientation.y = response[1][1]
        posemsg.orientation.z = response[1][2]
#        posemsg.orientation.w = response[1][3] #Commenting out to test rpy from gui

        speedmsg = Float32()
        speedmsg.data = response[2]

        print("--ROSLOG--")
        rospy.loginfo(posemsg)
        rospy.loginfo(speedmsg)

        pose_pub.publish(posemsg)
        speed_pub.publish(speedmsg)
        rate.sleep()

        if args.loop:
            time.sleep(1.0)

            # Don't repeat commands
            if 'forward' in request:
                moveForwardVal = 0
                del request['forward']
            if 'speed' in request:
                speedVal = 0
                del request['speed']
            if 'pose' in request:
                pose = None
                del request['pose']

        else:
            # Exit the loop.
            break


if __name__ == '__main__':
    main()
