#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import matplotlib as mpl
mpl.use("TkAgg") # Use TKAgg to show figures
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # plt.show()
        # plt.bar(left = 0,height = 1)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass