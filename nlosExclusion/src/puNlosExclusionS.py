#!/usr/bin/env python
# license removed for brevity
"""
    Nlos exclusion show
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me
"""
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as  NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd # pandas to pd
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import sys
import  math
from matplotlib.patches import Circle
import csv # csv reading needed library
from nlosExclusion.msg import GNSS_Raw,GNSS_Raw_Array # ros msg
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # ros message needed
import puCSV2Topi
from PyQt4 import QtCore, QtGui
import puNlosExclusion
import puGNSSPosCal
import time
class PlotcurveEval(QMainWindow): # plot curve
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('puCurve')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        # self.axes_1.clear()
        # self.axes_1.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        # self.axes = self.fig.add_subplot(111, projection='3d')
        self.axesCurv_1 = self.fig.add_subplot(311)
        self.axesCurv_2 = self.fig.add_subplot(312)
        self.axesCurv_3 = self.fig.add_subplot(313)
        # self.axes.get_zaxis().set_visible(False)


        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

class puSkyplot(QMainWindow): # plot skyplot
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('puSkyplot')
        self.create_main_frame()
        self.fontSize = 20
        self.fontColor = 'g'
        # self.drawBase()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        # self.axes_1.clear()
        # self.axes_1.grid(True)
        self.canvas.draw()

    def drawBase(self): # draw base for skyplot
        self.axes_2.set_aspect('equal')
        self.axes_2.add_artist(Circle((0, 0), 50, color='lightgrey'))  # self.circle = Circle((0, 0), 50)
        self.axes_2.add_artist(Circle((0, 0), 40, color='silver'))
        self.axes_2.add_artist(Circle((0, 0), 30, color='darkgray'))
        self.axes_2.add_artist(Circle((0, 0), 20, color='gray'))
        self.axes_2.add_artist(Circle((0, 0), 10, color='dimgrey'))
        self.axes_2.plot([0, 50], [0, 0], linewidth='1', color='gray')  # draw a line from (0,0) to (50,0)
        self.axes_2.text(51, 0, 'E', fontdict={'size': self.fontSize, 'color': self.fontColor})  # draw the 'E'
        self.axes_2.plot([0, 0], [0, 50], linewidth='1', color='gray')  # draw a line from (0,0) to (0,50)
        self.axes_2.text(0, 51, 'N', fontdict={'size': self.fontSize, 'color': self.fontColor})  # draw the 'N'
        self.axes_2.plot([0, -50], [0, 0], linewidth='1', color='gray')  # draw a line from (0,0) to (-50,0)
        self.axes_2.text(-54, 0, 'W', fontdict={'size': self.fontSize, 'color': self.fontColor})  # draw the 'W'
        self.axes_2.plot([0, 0], [0, -50], linewidth='1', color='gray')  # draw a line from (0,0) to (0,-50)
        self.axes_2.text(0, -54, 'S', fontdict={'size': self.fontSize, 'color': self.fontColor})  # draw the 'S'
        # tilt and azimuth
        self.axes_2.plot([0, 43.3], [0, 25], linewidth='1', color='gray')
        self.axes_2.text(45, 26, '60', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, 25], [0, 43.3], linewidth='1', color='gray')
        self.axes_2.text(26, 45, '30', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, -43.3], [0, 25], linewidth='1', color='gray')
        self.axes_2.text(-51, 28, '300', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, -25], [0, 43.3], linewidth='1', color='gray')
        self.axes_2.text(-27, 48, '330', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, -43.3], [0, -25], linewidth='1', color='gray')
        self.axes_2.text(-51, -30, '240', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, -25], [0, -43.3], linewidth='1', color='gray')
        self.axes_2.text(-29, -48, '210', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, 43.3], [0, -25], linewidth='1', color='gray')
        self.axes_2.text(48, -28, '120', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.plot([0, 25], [0, -43.3], linewidth='1', color='gray')
        self.axes_2.text(27, -47, '150', fontdict={'size': self.fontSize, 'color': self.fontColor})
        # draw elevation indicators
        self.axes_2.text(2, 13, '72', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.text(2, 23, '54', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.text(2, 33, '36', fontdict={'size': self.fontSize, 'color': self.fontColor})
        self.axes_2.text(2, 43, '18', fontdict={'size': self.fontSize, 'color': self.fontColor})

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes_2 = self.fig.add_subplot(111)
        self.axes_2.set_xlim([-60, 60])
        self.axes_2.set_ylim([-60, 60])
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

class puRostVisSample(PlotcurveEval): # plot curve sample
    def __init__(self):
        PlotcurveEval.__init__(self)
        self.window_size=20
        self.subscriber=rospy.Subscriber("ivsicklidar_", LaserScan, self.callback)


    def callback(self, data):
        print 'call back...'
        theta = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        z=np.ones(len(theta))
        print len(theta),len(data.ranges),type(theta),type(data.ranges)
        self.axesCurv_1.plot(theta, data.ranges,color='red',linewidth=2)
        self.axesCurv_2.plot(theta, data.ranges, color='red', linewidth=2)
        self.canvas.draw()
        plt.pause(0.001)
        self.axesCurv_1.clear()
        self.axesCurv_2.clear()

class puDouDeckBusS(puSkyplot): # callback double-decker
    def __init__(self):
        puSkyplot.__init__(self)
        self.window_size=20
        self.calAngNS = -105.57  # define calibrationa angle of car yaw and earth north
        rospy.Subscriber('double_decker_parameters', PoseArray,self.callDouDeckBusS)  # initialize subscriber to "double_decker_parameters", which contains azimuth and elevetion of double-decker bus boundary
        rospy.Subscriber('GNSSNlos_', GNSS_Raw_Array, self.CallGNSS_1)
        self.azim_ = []
        self.elev_ = []
        self.satIdx = []
        self.bouSide1X = []
        self.bouSide1Y = []
        self.bouSide2X = []
        self.bouSide2Y = []
        self.bouSide1X_ = []
        self.bouSide1Y_ = []
        self.bouSide2X_ = []
        self.bouSide2Y_ = []
        self.posArr = []  # create a list to save double-decker bus boundary information
        self.satColor='b'
        self.satColorLos  = 'g'
        self.satColorNlos = 'r'
        self.maxCircRadi = 50

    def callDouDeckBusS(self, data):
        self.posArr = data.poses  # save double-decker bus boundary information

        # print 'callDouble-decker S...'
        for i in range(len(self.posArr)):
            # x--azimuth y--elevation
            if (self.posArr[i].orientation.x < 0):
                self.posArr[i].orientation.x = self.posArr[i].orientation.x + 360
            if (self.posArr[i].orientation.z < 0):
                self.posArr[i].orientation.z = self.posArr[i].orientation.z + 360
            self.bouSide1X.append(-1 * (self.posArr[i].orientation.y * (-0.55556) + 50.0) * np.cos((self.posArr[
                                                                                                       i].orientation.x - 90.0 + self.calAngNS) * 3.14159 / 180.0))  # start point azimuth
            self.bouSide1Y.append((self.posArr[i].orientation.y * (-0.55556) + 50.0) * np.sin(-1 * (self.posArr[
                                                                                                       i].orientation.x - 90.0 + self.calAngNS) * 3.14159 / 180.0))  # start point elevation
            self.bouSide2X.append(-1 * (self.posArr[i].orientation.w * (-0.55556) + 50.0) * np.cos(-1 * (
                    self.posArr[
                        i].orientation.z - 90.0 + self.calAngNS) * 3.14159 / 180.0))  # end point azimuth
            self.bouSide2Y.append((self.posArr[i].orientation.w * (-0.55556) + 50.0) * np.sin(-1 * (self.posArr[
                                                                                                       i].orientation.z - 90.0 + self.calAngNS) * 3.14159 / 180.0))  # end point elevation
            uniVecSid1X_= self.bouSide1X[i] / (math.sqrt(self.bouSide1X[i] * self.bouSide1X[i] + self.bouSide1Y[i] * self.bouSide1Y[i]))
            uniVecSid1Y_ = self.bouSide1Y[i] / (math.sqrt(self.bouSide1X[i] * self.bouSide1X[i] + self.bouSide1Y[i] * self.bouSide1Y[i]))
            uniVecSid2X_ = self.bouSide2X[i] / (math.sqrt(self.bouSide2X[i] * self.bouSide2X[i] + self.bouSide2Y[i] * self.bouSide2Y[i]))
            uniVecSid2Y_ = self.bouSide2Y[i] / (math.sqrt(self.bouSide2X[i] * self.bouSide2X[i] + self.bouSide2Y[i] * self.bouSide2Y[i]))
            self.bouSide1X_.append(uniVecSid1X_ * self.maxCircRadi)
            self.bouSide1Y_.append(uniVecSid1Y_ * self.maxCircRadi)
            self.bouSide2X_.append(uniVecSid2X_ * self.maxCircRadi)
            self.bouSide2Y_.append(uniVecSid2Y_ * self.maxCircRadi)
        # self.drawBase()
        # self.canvas.draw()
        # # plt.pause(0.001)
        # self.cleanS()

    def CallGNSS_1(self, data):  # GNSS data
        self.GNSS_1 = GNSS_Raw_Array()
        self.GNSS_1 = data
        self.satecirclRad = 3
        for index_1 in range(len(self.GNSS_1.GNSS_Raws)):  # index all the satellite information in one epoch
            self.azim_.append( (self.GNSS_1.GNSS_Raws[index_1].elevation*(-0.55556)+50.0)* np.cos(-1*(self.GNSS_1.GNSS_Raws[index_1].azimuth-90.0)*3.14159/180.0))
            self.elev_.append( (self.GNSS_1.GNSS_Raws[index_1].elevation*(-0.55556)+50.0)* np.sin(-1*(self.GNSS_1.GNSS_Raws[index_1].azimuth-90.0)*3.14159/180.0))
            self.satIdx.append(self.GNSS_1.GNSS_Raws[index_1].prn_satellites_index)
        self.drawAll()

    def drawAll(self):
        self.drawBase()
        for sate_index in range(len(self.azim_)):  # draw the satellite into the Skyplot (template: circle+G30)
            if(self.GNSS_1.GNSS_Raws[sate_index].visable == 2):
                self.axes_2.add_artist(Circle((self.azim_[sate_index], self.elev_[sate_index]), self.satecirclRad,
                                              color=self.satColorNlos))  # self.circle = Circle((0, 0), 50)
                self.axes_2.text(self.azim_[sate_index], self.elev_[sate_index], str(int(self.satIdx[sate_index])),
                                 fontdict={'size': self.fontSize, 'color': self.satColor})  # draw the 'E'
            else:
                self.axes_2.add_artist(Circle((self.azim_[sate_index], self.elev_[sate_index]), self.satecirclRad, color=self.satColorLos))  # self.circle = Circle((0, 0), 50)
                self.axes_2.text(self.azim_[sate_index], self.elev_[sate_index], str(int(self.satIdx[sate_index])), fontdict={'size': self.fontSize, 'color': self.satColor})  # draw the 'E'
        self.axes_2.plot([self.bouSide1X, self.bouSide2X], [self.bouSide1Y, self.bouSide2Y], linewidth='2', color='fuchsia')  # draw a line from (0,0) to (50,0)
        self.axes_2.plot([self.bouSide1X, self.bouSide1X_], [self.bouSide1Y, self.bouSide1Y_], linewidth='1',color='fuchsia')  # draw a line from (0,0) to (50,0)
        self.axes_2.plot([self.bouSide2X, self.bouSide2X_], [self.bouSide2Y, self.bouSide2Y_], linewidth='1',color='fuchsia')  # draw a line from (0,0) to (50,0)
        self.canvas.draw()
        self.cleanS()

    def cleanS(self):
        self.axes_2.clear()
        # self.bouSide1X[:]
        # self.bouSide1Y[:]
        # self.bouSide2X[:]
        # self.bouSide2Y[:]
        # self.bouSide1X_[:]
        # self.bouSide1Y_[:]
        # self.bouSide2X_[:]
        # self.bouSide2Y_[:]
        # self.azim_[:]
        # self.elev_[:]
        # self.satIdx[:]
        self.bouSide1X[:] = [] 
        self.bouSide1Y[:] = []
        self.bouSide2X[:] = []
        self.bouSide2Y[:] = []
        self.bouSide1X_[:] = []
        self.bouSide1Y_[:] = []
        self.bouSide2X_[:] = []
        self.bouSide2Y_[:] = []
        self.azim_[:] = []
        self.elev_[:] = []
        self.satIdx[:] = []

class readCSV2TopiThread(QtCore.QThread):
    """docstring for readCSV2TopiThread"""
    def __init__(self, parent=None):
        super(readCSV2TopiThread, self).__init__(parent)
        self.GNSS_ = GNSS_Raw_Array()
    def run(self):
        self.puCsv2Topic_ = puCSV2Topi.csv2Topi('/home/wenws/3_nlosMovobj/src/nlosExclusion/src/data5/exper2/sv_data.csv', 1)
        self.GNSS_= self.puCsv2Topic_.GNSSArr_

class nlosExclThread(QtCore.QThread):
    """docstring for readCSV2TopiThread"""
    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent)
        self.posArr = []
        self.GNSS_ = GNSS_Raw_Array()
        self.print_ = 8
        self.iniSatLis_ = []
        self.iniSnr = []
        # self.excluSatLis = [29,23,92] # for small nlos, experiment 1
        self.excluSatLis = [3,91,22,32,94,23,26,93] #  23,26,93 for big nlos, experiment 2 (alway visible: 13 26 31 93 96 100     sometimes visible: 23) Low elevation:3,91,22,32,94,
        self.nlosExclusionF_ = puNlosExclusion.nlosExclusion()
        self.puGNSSPosCalF_ =puGNSSPosCal.GNSSPosCal()
        self.puGNSSPosCalF2_ = puGNSSPosCal.GNSSPosCal()
        rospy.Subscriber('double_decker_parameters', PoseArray, self.callDouDeckBus)
        rospy.Subscriber('GNSS_', GNSS_Raw_Array, self.CallGNSS_)


    def callDouDeckBus(self, data):  # double-decker bus data
        self.posArr = data.poses  # save double-decker bus boundary information
        # print 'callDouble-decker...'

    def CallGNSS_(self, data):  # GNSS data

        self.GNSS_ = data
        self.iniSnr[:] = []
        self.iniSatLis_[:] = []
        for idx in range(len(self.GNSS_.GNSS_Raws)):
            self.iniSatLis_.append(self.GNSS_.GNSS_Raws[idx].prn_satellites_index)
            self.iniSnr.append(self.GNSS_.GNSS_Raws[idx].snr)
        # print 'GNSS_ ...'

    def nlosExclusionF(self):  # function
        # print len(self.posArr),len(self.GNSS_.GNSS_Raws)
        if (len(self.posArr) > 0) and (len(self.GNSS_.GNSS_Raws) > 0):
            # print 'both double-decker bus and GNSS_ are ready...'
            self.nlosExclusionF_ = puNlosExclusion.nlosExclusion()
            self.nlosExclusionF_.nlosExclusion_(self.posArr, self.GNSS_,'statManu',self.excluSatLis)
            self.puGNSSPosCalF_ = puGNSSPosCal.GNSSPosCal()
            # self.puGNSSPosCalF_.iterPosCal(self.GNSS_,'LSGNSS')
            self.puGNSSPosCalF_.iterPosCal(self.nlosExclusionF_.IniGNSS_, 'WLSGNSS')
            self.puGNSSPosCalF2_ = puGNSSPosCal.GNSSPosCal()
            self.puGNSSPosCalF2_.iterPosCal(self.nlosExclusionF_.nlosGNSSDel_, 'WLSGNSS')
            self.posArr[:] = []
            self.GNSS_.GNSS_Raws[:] = []
    def run(self):
        while(1):
            # print 'threda'
            time.sleep(0.2)
            self.nlosExclusionF()


class nlosExclusionS(QMainWindow): # by timer

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('evaluation')
        self.GNSSTim_ = []
        self.GNSSTimUpd = 0
        self.totaSat_ = []
        self.totaSatExcl_ = []
        self.GPSSatNum_ = []
        self.BeiSatNum_ = []
        self.GPSExclNum_ = []
        self.BeiExclNum_ = []
        self.hdop_ = []
        self.hdop2_ = []
        self.errorCon_ = []
        self.errorProp_ = []
        self.satVis_ = []
        self.snr_ = []
        self.create_main_frame()
        self.on_draw()
        self.csv2TopicThr_ = readCSV2TopiThread()
        self.csv2TopicThr_.start()
        self.nlosExclThra_=nlosExclThread()
        self.nlosExclThra_.start()
        print 'open two thread'
        self.prinInfo()
        self.calAngN = -105.57  # define calibrationa angle of car yaw and earth north
        self.posArr = []  # create a list to save double-decker bus boundary information
        self.timer = QtCore.QTimer()
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self.prinInfo)
        self.timer.start(100)

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        # self.axes_1.clear()
        # self.axes_1.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 20.0), dpi=self.dpi) # 5.0 4.0
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        # self.axes = self.fig.add_subplot(111, projection='3d')
        self.axesCurv_1 = self.fig.add_subplot(411)
        self.axesCurv_2 = self.fig.add_subplot(412)
        self.axesCurv_3 = self.fig.add_subplot(413)
        self.axesCurv_4 = self.fig.add_subplot(414)
        # self.axes.get_zaxis().set_visible(False)


        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

    def prinInfo(self):
        if(self.GNSSTimUpd != self.nlosExclThra_.nlosExclusionF_.GNSSTime):
            self.GNSSTimUpd = self.nlosExclThra_.nlosExclusionF_.GNSSTime
            self.GNSSTim_.append(self.nlosExclThra_.nlosExclusionF_.GNSSTime)
            self.totaSat_.append(len(self.nlosExclThra_.nlosExclusionF_.satIdx))
            self.GPSSatNum_.append(self.nlosExclThra_.nlosExclusionF_.GPSNum_)
            self.BeiSatNum_.append(self.nlosExclThra_.nlosExclusionF_.BeiNum_)
            self.GPSExclNum_.append(self.nlosExclThra_.nlosExclusionF_.GPSExclNum)
            self.BeiExclNum_.append(self.nlosExclThra_.nlosExclusionF_.BeidExclNum)
            self.hdop_.append(self.nlosExclThra_.puGNSSPosCalF_.dop)
            self.hdop2_.append(self.nlosExclThra_.puGNSSPosCalF2_.dop)
            self.errorCon_.append(self.nlosExclThra_.puGNSSPosCalF_.error_)
            self.errorProp_.append(self.nlosExclThra_.puGNSSPosCalF2_.error_)
            if(self.nlosExclThra_.excluSatLis[-1] in self.nlosExclThra_.iniSatLis_):
                self.satVis_.append(float(20.0))
                snrIdx = self.nlosExclThra_.iniSatLis_.index(self.nlosExclThra_.excluSatLis[-1])
                self.snr_.append(self.nlosExclThra_.iniSnr[snrIdx])
            if (self.nlosExclThra_.excluSatLis[-1] not in self.nlosExclThra_.iniSatLis_):
                self.satVis_.append(float(0.0))
                self.snr_.append(0.0)
            self.nlosExclThra_.iniSatLis_[:]= []
            # print 'self.nlosExclThra_.iniSnr[:]' ,self.nlosExclThra_.iniSnr
            self.nlosExclThra_.iniSnr[:] = []
            print 'update...'
        else:
            print ' not update...'
        # self.axesCurv_1.plot(self.nlosExclThra_.nlosExclusionF_.azim_, self.nlosExclThra_.nlosExclusionF_.elev_, '*',linewidth='1',
        #                  color='fuchsia')
        self.axesCurv_1.plot(self.GNSSTim_, self.totaSat_, '-*',linewidth='1',color='black',label='TotalSv')
        self.axesCurv_1.plot(self.GNSSTim_, self.GPSSatNum_, '-*',linewidth='1', color='deepskyblue',label='GPSSv')
        self.axesCurv_1.plot(self.GNSSTim_, self.BeiSatNum_, '-*', linewidth='1', color='fuchsia', label='BeiSv')
        self.axesCurv_1.plot(self.GNSSTim_, self.GPSExclNum_, '-*', linewidth='1', color='lime', label='GPSExclSv')
        self.axesCurv_1.plot(self.GNSSTim_, self.BeiExclNum_, '-*', linewidth='1', color='dodgerblue', label='BeiExclSv')
        self.axesCurv_1.legend(loc=9, ncol=5, shadow=True)
        self.axesCurv_2.plot(self.GNSSTim_, self.hdop_, '-*', linewidth='1', color='dodgerblue',label='dop')
        self.axesCurv_2.plot(self.GNSSTim_, self.hdop2_, '-*', linewidth='1', color='lime', label='proposed dop')
        self.axesCurv_2.legend(loc=9, ncol=5, shadow=True)
        self.axesCurv_3.plot(self.GNSSTim_, self.errorCon_, '-*', linewidth='1', color='dodgerblue', label='positioning error')
        self.axesCurv_3.plot(self.GNSSTim_, self.errorProp_, '-*', linewidth='1', color='lime',label='proposed positioning error')
        if(len(self.errorProp_)>0):
            meanEr_ = []
            std_ = np.std(self.errorCon_)
            meanEr_.append(sum(self.errorCon_)/(len(self.errorCon_)))
            print 'meanEr_',sum(self.errorCon_)/(len(self.errorCon_)),'std_----',std_
            self.PercentCal(self.errorCon_)
            meanEr_ = meanEr_ * len(self.GNSSTim_)
            # self.axesCurv_3.plot(self.GNSSTim_, meanEr_, '-*', linewidth='3', color='dodgerblue',label='mean error')
            meanErPros_ = []
            stdProp_ = np.std(self.errorProp_)
            meanErPros_.append(sum(self.errorProp_)/(len(self.errorProp_)))
            print 'meanErPros_',sum(self.errorProp_)/(len(self.errorProp_)),'stdProp_',stdProp_
            self.PercentCal(self.errorProp_)
            meanErPros_ = meanErPros_ * len(self.GNSSTim_)
            # self.axesCurv_3.plot(self.GNSSTim_, meanErPros_, '-*', linewidth='3', color='lime', label='proposed mean error')
            print 'mean hdop',np.mean(self.hdop_),'hdop std',np.std(self.hdop_)
            print 'mean proposed hdop',np.mean(self.hdop2_),'proposed hdop std',np.std(self.hdop2_)
        self.axesCurv_3.legend(loc=9, ncol=5, shadow=True)
        self.axesCurv_4.plot(self.GNSSTim_, self.satVis_, '-o', linewidth='1', color='dodgerblue', label='visibility')
        self.axesCurv_4.plot(self.GNSSTim_, self.snr_, '-*', linewidth='2', color='lime', label='SNR')
        self.axesCurv_4.legend(loc=9, ncol=5, shadow=True)
        self.axesCurv_4.legend(loc=9, ncol=5, shadow=True)

        self.canvas.draw()
        self.axesCurv_1.clear()
        self.axesCurv_2.clear()
        self.axesCurv_3.clear()
        self.axesCurv_4.clear()
        # print 'self.GNSSTim_',self.GNSSTim_
    def PercentCal(self,data_):
        # self.axes_1.grid(True)
        error_ = []
        error_ = data_
        percent_1 = 0.0 # >15
        percent_2 = 0.0 # >25
        percent_3 = 0.0 # >40
        for perCal in range(len(error_)):
            if(error_[perCal]<=15):
                percent_1 = percent_1 + 1
            if (error_[perCal] <= 30):
                percent_2 = percent_2 + 1
            if (error_[perCal] >= 40):
                percent_3 = percent_3 + 1
        percent_1 = percent_1 / len(error_)
        percent_2 = percent_2 / len(error_)
        percent_3 = percent_3 / len(error_)
        print 'percent_1=',percent_1,'percent_2=',percent_2,'percent_3=',percent_3

class nlosExclusionS_(QMainWindow): # by timer   For paper NLOS exclusion caused by double-decekr bus

    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('evaluation')
        self.GNSSTim_ = []
        self.GNSSTimUpd = 0
        self.totaSat_ = []
        self.totaSatExcl_ = []
        self.GPSSatNum_ = []
        self.BeiSatNum_ = []
        self.GPSExclNum_ = []
        self.BeiExclNum_ = []
        self.hdop_ = []
        self.hdop2_ = []
        self.errorCon_ = []
        self.errorProp_ = []
        self.satVis_ = []
        self.snr_ = []
        self.create_main_frame()
        self.on_draw()
        self.csv2TopicThr_ = readCSV2TopiThread()
        self.csv2TopicThr_.start()
        self.nlosExclThra_=nlosExclThread()
        self.nlosExclThra_.start()
        print 'open two thread'
        self.prinInfo()
        self.calAngN = -105.57  # define calibrationa angle of car yaw and earth north
        self.posArr = []  # create a list to save double-decker bus boundary information
        self.timer = QtCore.QTimer()
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self.prinInfo)
        self.timer.start(100)

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        # self.axes_1.clear()
        # self.axes_1.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 20.0), dpi=self.dpi,facecolor='none') # 5.0 4.0
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        # self.axes = self.fig.add_subplot(111, projection='3d')
        self.axesCurv_1 = self.fig.add_subplot(311)
        self.axesCurv_2 = self.fig.add_subplot(312)
        self.axesCurv_3 = self.fig.add_subplot(313)
        # self.axes.get_zaxis().set_visible(False)


        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

    def prinInfo(self):
        if(self.GNSSTimUpd != self.nlosExclThra_.nlosExclusionF_.GNSSTime):
            self.GNSSTimUpd = self.nlosExclThra_.nlosExclusionF_.GNSSTime
            self.GNSSTim_.append(self.nlosExclThra_.nlosExclusionF_.GNSSTime - 131519)
            self.totaSat_.append(len(self.nlosExclThra_.nlosExclusionF_.satIdx))
            self.GPSSatNum_.append(self.nlosExclThra_.nlosExclusionF_.GPSNum_)
            self.BeiSatNum_.append(self.nlosExclThra_.nlosExclusionF_.BeiNum_)
            self.GPSExclNum_.append(self.nlosExclThra_.nlosExclusionF_.GPSExclNum)
            self.BeiExclNum_.append(self.nlosExclThra_.nlosExclusionF_.BeidExclNum)
            totaSVExc_ = 0.0
            totaSVExc_ = len(self.nlosExclThra_.nlosExclusionF_.satIdx) - self.nlosExclThra_.nlosExclusionF_.GPSExclNum- self.nlosExclThra_.nlosExclusionF_.BeidExclNum
            if(totaSVExc_ < 6 ):
                totaSVExc_ = 6
            self.totaSatExcl_.append(totaSVExc_)
            self.hdop_.append(self.nlosExclThra_.puGNSSPosCalF_.dop)
            self.hdop2_.append(self.nlosExclThra_.puGNSSPosCalF2_.dop)
            self.errorCon_.append(self.nlosExclThra_.puGNSSPosCalF_.error_)
            self.errorProp_.append(self.nlosExclThra_.puGNSSPosCalF2_.error_)
            if(self.nlosExclThra_.excluSatLis[-1] in self.nlosExclThra_.iniSatLis_):
                self.satVis_.append(float(20.0))
                snrIdx = self.nlosExclThra_.iniSatLis_.index(self.nlosExclThra_.excluSatLis[-1])
                self.snr_.append(self.nlosExclThra_.iniSnr[snrIdx])
            if (self.nlosExclThra_.excluSatLis[-1] not in self.nlosExclThra_.iniSatLis_):
                self.satVis_.append(float(0.0))
                self.snr_.append(0.0)
            self.nlosExclThra_.iniSatLis_[:]= []
            # print 'self.nlosExclThra_.iniSnr[:]' ,self.nlosExclThra_.iniSnr
            self.nlosExclThra_.iniSnr[:] = []
            print 'update...'
        else:
            print ' not update...'
        # self.axesCurv_1.plot(self.nlosExclThra_.nlosExclusionF_.azim_, self.nlosExclThra_.nlosExclusionF_.elev_, '*',linewidth='1',
        #                  color='fuchsia')
        self.axesCurv_1.plot(self.GNSSTim_, self.totaSat_, '-*',linewidth='1',color='red',label='TotalSv')
        # self.axesCurv_1.set_xlabel('sss',fontsize=24)
        self.axesCurv_1.tick_params(axis='both',labelsize = 25)
        self.axesCurv_2.tick_params(axis='both', labelsize= 25)
        self.axesCurv_3.tick_params(axis='both', labelsize= 25)
        # self.axesCurv_1.plot(self.GNSSTim_, self.GPSSatNum_, '-*',linewidth='1', color='deepskyblue',label='GPSSv')
        # self.axesCurv_1.plot(self.GNSSTim_, self.BeiSatNum_, '-*', linewidth='1', color='fuchsia', label='BeiSv')
        # self.axesCurv_1.plot(self.GNSSTim_, self.GPSExclNum_, '-*', linewidth='1', color='lime', label='GPSExclSv')
        # self.axesCurv_1.plot(self.GNSSTim_, self.BeiExclNum_, '-*', linewidth='1', color='dodgerblue', label='BeiExclSv')
        self.axesCurv_1.plot(self.GNSSTim_, self.totaSatExcl_, '-*', linewidth='1', color='blue', label=' proposed TotalSv')
        self.axesCurv_2.plot(self.GNSSTim_, self.hdop_, '-*', linewidth='1', color='red',label='dop')
        self.axesCurv_2.plot(self.GNSSTim_, self.hdop2_, '-*', linewidth='1', color='blue', label='proposed dop')
        self.axesCurv_3.plot(self.GNSSTim_, self.errorCon_, '-*', linewidth='1', color='red', label='positioning error')
        self.axesCurv_3.plot(self.GNSSTim_, self.errorProp_, '-*', linewidth='1', color='blue',label='proposed positioning error')
        if(len(self.errorProp_)>0):
            meanEr_ = []
            std_ = np.std(self.errorCon_)
            meanEr_.append(sum(self.errorCon_)/(len(self.errorCon_)))
            print 'meanEr_',sum(self.errorCon_)/(len(self.errorCon_)),'std_----',std_
            self.PercentCal(self.errorCon_)
            meanEr_ = meanEr_ * len(self.GNSSTim_)
            # self.axesCurv_3.plot(self.GNSSTim_, meanEr_, '-*', linewidth='3', color='dodgerblue',label='mean error')
            meanErPros_ = []
            stdProp_ = np.std(self.errorProp_)
            meanErPros_.append(sum(self.errorProp_)/(len(self.errorProp_)))
            print 'meanErPros_',sum(self.errorProp_)/(len(self.errorProp_)),'stdProp_',stdProp_
            self.PercentCal(self.errorProp_)
            meanErPros_ = meanErPros_ * len(self.GNSSTim_)
            # self.axesCurv_3.plot(self.GNSSTim_, meanErPros_, '-*', linewidth='3', color='lime', label='proposed mean error')
            print 'mean hdop',np.mean(self.hdop_),'hdop std',np.std(self.hdop_)
            print 'mean proposed hdop',np.mean(self.hdop2_),'proposed hdop std',np.std(self.hdop2_)

        self.canvas.draw()
        self.axesCurv_1.clear()
        self.axesCurv_2.clear()
        self.axesCurv_3.clear()
        # print 'self.GNSSTim_',self.GNSSTim_
    def PercentCal(self,data_):
        # self.axes_1.grid(True)
        error_ = []
        error_ = data_
        percent_1 = 0.0 # >15
        percent_2 = 0.0 # >25
        percent_3 = 0.0 # >40
        for perCal in range(len(error_)):
            if(error_[perCal]<=15):
                percent_1 = percent_1 + 1
            if (error_[perCal] <= 30):
                percent_2 = percent_2 + 1
            if (error_[perCal] >= 40):
                percent_3 = percent_3 + 1
        percent_1 = percent_1 / len(error_)
        percent_2 = percent_2 / len(error_)
        percent_3 = percent_3 / len(error_)
        print 'percent_1=',percent_1,'percent_2=',percent_2,'percent_3=',percent_3

if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('puNlosExclusion', anonymous=True)
    # puCurve = puRostVisSample()
    # puCurve.show()
    puDouDeckBusS_=puDouDeckBusS()
    puDouDeckBusS_.show()
    nlosExclusion_=nlosExclusionS_()
    nlosExclusion_.show()
    app.exec_()