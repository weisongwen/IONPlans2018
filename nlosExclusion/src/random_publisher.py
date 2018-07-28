#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int8
def talker():
    pub = rospy.Publisher('random_value', Int8)
    rospy.init_node('random_value')
    while not rospy.is_shutdown():
        pub.publish(random.random()*10)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
