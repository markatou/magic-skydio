import rospy
import json
from std_msgs.msg import String

from http_client import HTTPClient

global execute
execute = False

global path
path = []

global motions
motions = []

global userabort
userabort = False

WAYPOINT_TABLE = {'tripod': [3.0, 3.0, 3.0], 'chair': [0.0, 0.0, 0.0], 'door': [0.5, 0.0, 1.1]}

def record_callback(msg):
    global execute

    speech_to_path(msg.data)
    if len(path) > 0:
        execute = True

def speech_to_path(command):
    global path
    global motions
    global userabort

    parsed = command.split(' ')
    for p in parsed:
        if p in WAYPOINT_TABLE.keys():
            path.append(WAYPOINT_TABLE[p])
        if p == "abort":
            userabort = True

def main():
    global execute 
    global path
    global userabort
    
    # Create the client to use for all requests.
    client = HTTPClient('https://sim2-0.sim-us-east.skydio.com',
                        pilot=False,
                        token_file='sim2token.txt')

    # Set R1 to waypoint skill
    # client.set_skill("apmdp.interiorwaypoints.LTLWaypoints")

    rospy.init_node('indoor_waypoints')
    rate = rospy.Rate(10)
    rospy.Subscriber("/pidrone/speech_final", String, record_callback)

    while not rospy.is_shutdown():
        request = {}
        if execute:
            request['move'] = path
        if userabort:
            request['abort'] = "USER ABORT REQUESTED" # By design, main.py will now need to be killed and restarted to run another mission
                 
        if execute or userabort:
            client.send_custom_comms("apmdp.interiorwaypoints.LTLWaypoints", json.dumps(request))
            
            # Reset request
            path = []
            execute = False

        rate.sleep()

if __name__ == "__main__":
    main()
