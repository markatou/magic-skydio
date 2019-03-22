#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

def pose_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + 'Position: %s', str(msg.position.x) + '| ' +
                                                        str(msg.position.y) + '| ' +
                                                        str(msg.position.z))
    
    rospy.loginfo(rospy.get_caller_id() + 'Orientation: %s', str(msg.orientation.x) + '| ' +
                                                        str(msg.orientation.y) + '| ' + 
                                                        str(msg.orientation.z) + '| ' + 
                                                        str(msg.orientation.w))

def speed_callback(msg):
    rospy.loginfo(rospy.get_caller_id() + 'Speed: %s', str(msg.data))

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('pose', Pose, pose_callback)
    rospy.Subscriber('speed', Float32, speed_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()