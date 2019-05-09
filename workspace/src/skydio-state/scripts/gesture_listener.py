from __future__ import absolute_import
from http_client import HTTPClient
import argparse
import time
import threading
import os
import rospy
from std_msgs.msg import String

global numOkay
numOkay = 0

def gesture_callback(msg):
    global numOkay
    if msg.data == "Okay":
        numOkay += 1
    else:
        numOkay = 0


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
    
    args = parser.parse_args()

    # Create the client to use for all requests.
    client = HTTPClient(args.baseurl,
                        pilot=True,
                        token_file=args.token_file)
    
    #Pilot stuff
    def update_loop():
        while True:
            client.update_pilot_status()
            time.sleep(2)
    status_thread = threading.Thread(target=update_loop)
    status_thread.setDaemon(True)
    status_thread.start() 

    # Initializing node
    rospy.init_node('gesture_receiver', anonymous=True)

    # Listen to movement publishers (for control support)
    rospy.Subscriber('gesture', String, gesture_callback)

    status_pub = rospy.Publisher('activate', String, queue_size=1)

    while not rospy.is_shutdown():
        if numOkay > 10:
            print("setting skill")
            client.set_skill(args.skill_key)
            statusmsg = String("skill set")
            status_pub.publish(statusmsg)


if __name__ == '__main__':
    main()