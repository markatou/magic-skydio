#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

def talker():
    pub = rospy.Publisher('go_forward', Int64, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Sends a distance to go_forward topic once
    distance = 1 # Modify me to change distance moved forward
    rospy.loginfo("Published" + str(distance))
    pub.publish(distance)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass