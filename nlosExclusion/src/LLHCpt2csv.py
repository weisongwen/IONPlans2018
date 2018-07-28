#!/usr/bin/env python
# license removed for brevity
"""
    Reads customized RINEX2 or RINEX3 file into ros topic
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me
    this file is finished for guohao
"""
import rospy #ros python needed
from std_msgs.msg import String #std message
from numpy import * # numpy needed
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs
import csv # csv reading needed library
import datetime #time format (datetime)
import time #time format (time)
from novatel_msgs.msg import BESTPOS
#read csv file into rostopic (GNSS Raw data)
filename='/home/wenws/llggllgllgllg.csv' # positioning result with NLOS exclusion
gps_week = []
gps_second = []
list_lat=[] #create a list to save latitude from recalculated GNSS positioning (with NLOS exclusion)
list_lon=[] #create a list to save longitude from recalculated GNSS positioning (with NLOS exclusion)
altitude = []
def recordLLH2CSV():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('recordLLH2CSV_', anonymous=True) #initialize node with specific node name
    rospy.Subscriber('/novatel_data/bestpos', BESTPOS, callcptLLh)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello_str = "current time is %s" % rospy.get_time()
        rate.sleep()

def callcptLLh(data):
    llhCpt_ = BESTPOS()
    llhCpt_ = data
    gps_week.append(llhCpt_.header.gps_week)
    gps_second.append(llhCpt_.header.gps_week_seconds)
    list_lat.append(llhCpt_.latitude)
    list_lon.append(float(llhCpt_.longitude))
    altitude.append(float(llhCpt_.altitude))
    if(len(list_lon) >  18):
        with open(filename, 'w') as file_object:  # save listLatRaw and listLonRaw to csv file
            for sav_idx in range(len(list_lat)):
                str2 = str(gps_week[sav_idx]) + ','+str(gps_second[sav_idx]) + ','+str(list_lat[sav_idx]) + ',' + str(list_lon[sav_idx])+',' + str(altitude[sav_idx])
                print str2
                file_object.write(str2)
                file_object.write('\n')
    print 'llhCpt_',llhCpt_.longitude,type(llhCpt_.longitude)

if __name__ == '__main__':
    recordLLH2CSV()