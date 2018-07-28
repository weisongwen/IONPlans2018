#!/usr/bin/env python
# license removed for brevity
"""
    CSV to Topic
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me
"""
import csv # csv reading needed library
import datetime #time format (datetime)
import time #time format (time)
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs
from nlosExclusion.msg import GNSS_Raw_Array,GNSS_Raw # customerized ros message type
from matplotlib.patches import Ellipse, Circle # draw circle needs library
import rospy #ros python needed
import ecef2llh #ecef coordinate to llh coordinate
from std_msgs.msg import String #std message
from numpy import * # numpy needed
import matplotlib as mpl #plot needed
mpl.use("TkAgg") # Use TKAgg to show figures:set this to show plot
import matplotlib.pyplot as plt #plotting
import pandas as pd # pandas needed renamed as pd
import numpy as np #numpy needed renamed as np
import geometry_msgs.msg as gm #ros geometry message
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # commonly used message type
from PyQt4 import QtCore, QtGui
import puGNSSPosCal
#read csv file into rostopic (GNSS Raw data)
class csv2Topi():
    def __init__(self,csvPath,tophz_):
        # rospy.init_node('csv2Topi_', anonymous=True)  # initialize node with specific node name
        self.GNSS_Raw_pub = rospy.Publisher('GNSS_', GNSS_Raw_Array,
                                       queue_size=10)  # customerized GNSS raw data ros message
        self.topHz =tophz_
        self.lowEleSatLis_ = [3, 91, 22, 32, 94]
        self.lowEleSatLis_ = []
        self.eleThres_ = 18.0 # 18
        self.snrThres_ = 20.0 # 20
        self.csvPath = csvPath # /home/wenws/3_nlosMovobj/src/nlosExclusion/src/data5/exper1/sv_data.csv
        if(tophz_>0):
            self.readCSV()
        self.GNSSArr_ = GNSS_Raw_Array()
    def readCSV(self):
        self.csv_GNSS = csv.reader(open(self.csvPath,
                                     'r'))  # read csv context to csv_reader variable
        self.GNSSArr    =GNSS_Raw_Array()
        self.GNSSTim    =0
        self.preGNSSTim =0
        self.linNum     =0
        self.preTopiTim = time.time()
        self.curTopTim  = time.time()
        for rowCsv in self.csv_GNSS:
            self.preTopiTim = float(round(time.time() * 1000))
            self.GNSS_       = GNSS_Raw()
            self.rosNavSaFix = NavSatFix()
            self.GNSSTim     =rowCsv[0]
            self.GNSS_.GNSS_time = float(rowCsv[0])  # GNSS time contained in csv file
            self.GNSS_.total_sv = float(rowCsv[1])  # total sitellites in one epoch (epoch: each time point)
            self.GNSS_.prn_satellites_index = float(
                rowCsv[2])  # satellite index in this epoch (epoch: each time point)
            self.GNSS_.pseudorange = float(rowCsv[3]) - float(rowCsv[7]) - float(rowCsv[8]) + float(
                rowCsv[9])  # pseudorange measurement
            self.GNSS_.snr = float(rowCsv[4])  # signal noise ratio for this satellite
            self.GNSS_.elevation = float(rowCsv[5])  # elevation for this satellite and receiver
            self.GNSS_.azimuth = float(rowCsv[6])  # azimuth for this satellite
            self.GNSS_.err_tropo = float(rowCsv[7])  # troposphere error in meters
            self.GNSS_.err_iono = float(rowCsv[8])  # ionophere error in meters
            self.GNSS_.sat_clk_err = float(rowCsv[9])  # satellite clock bias caused error
            self.GNSS_.sat_pos_x = float(rowCsv[10])  # satellite positioning in x direction
            self.GNSS_.sat_pos_y = float(rowCsv[11])  # satellite positioning in y direction
            self.GNSS_.sat_pos_z = float(rowCsv[12])  # satellite positioning in Z direction
            self.GNSS_.visable = 0  # satellite visability juded by NLOS exclusion
            if ((self.preGNSSTim == self.GNSSTim) or (self.linNum == 0)):
                if(self.GNSS_.elevation > self.eleThres_) and (self.GNSS_.snr > self.snrThres_) and (self.GNSS_.prn_satellites_index not in self.lowEleSatLis_):
                    self.GNSSArr.GNSS_Raws.append(self.GNSS_)
            else:
                self.GNSSArr.header.frame_id='GNSS_'
                self.GNSS_Raw_pub.publish(self.GNSSArr)
                # GNSSPosCal_    = puGNSSPosCal.GNSSPosCal()
                # GNSSPosCal_.iterPosCal(self.GNSSArr,'LSGNSS')
                # print 'llh',GNSSPosCal_.llh_,len(GNSSPosCal_.azimuth),GNSSPosCal_.azimuth
                self.GNSSArr_  = self.GNSSArr # keep for outside
                del self.GNSSArr.GNSS_Raws[:]
                if (self.GNSS_.elevation > self.eleThres_) and (self.GNSS_.snr > self.snrThres_) and (self.GNSS_.prn_satellites_index not in self.lowEleSatLis_):
                    self.GNSSArr.GNSS_Raws.append(self.GNSS_)
                self.curTopTim = float(round(time.time() * 1000))
                time.sleep(1/self.topHz-(self.curTopTim-self.preTopiTim)/1000.0)
                # time.sleep(1 / self.topHz )
                print 'rostopic time bias / ms ------------ ', self.curTopTim - self.preTopiTim
            self.preGNSSTim = self.GNSSTim
            self.linNum     = self.linNum+1
            # print 'linNum',self.linNum