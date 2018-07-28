#!/usr/bin/env python
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as  NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

class Plotcurve(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('Sliding histogramm')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        self.axes_1.clear()
        self.axes_1.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        # self.axes = self.fig.add_subplot(111, projection='3d')
        self.axes_1 = self.fig.add_subplot(311)
        self.axes_2 = self.fig.add_subplot(312)
        self.axes_3 = self.fig.add_subplot(313)
        # self.axes.get_zaxis().set_visible(False)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import sys

class plotnew_1(Plotcurve):
    def __init__(self):
        Plotcurve.__init__(self)
        self.window_size=20
        rospy.init_node('listener', anonymous=True)
        self.subscriber=rospy.Subscriber("ivsicklidar_", LaserScan, self.callback)


    def callback(self, data):
        print 'call back...'
        theta = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        z=np.ones(len(theta))
        print len(theta),len(data.ranges),type(theta),type(data.ranges)
        self.axes_1.plot(theta, data.ranges,color='red',linewidth=2)
        self.axes_2.plot(theta, data.ranges, color='red', linewidth=2)
        self.canvas.draw()
        plt.pause(0.001)
        self.axes_1.clear()
        self.axes_2.clear()
class plotnew_2(Plotcurve):
    def __init__(self):
        Plotcurve.__init__(self)
        self.window_size=20
        rospy.init_node('listener', anonymous=True)
        self.subscriber=rospy.Subscriber("ivsicklidar_", LaserScan, self.callback)
        self.x_1 = 0
        self.y_1 = 0
        self.list1=[]
        self.list2 = []



    def callback(self, data):
        print 'call back _2...'
        self.x_1=self.x_1+1
        self.y_1=self.y_1+1
        print 'self.x_1',self.x_1
        theta = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        z=np.ones(len(theta))
        print len(theta),len(data.ranges),type(theta),type(data.ranges)
        # self.axes.plot(theta, data.ranges,color='green',linewidth=2)
        self.list1.append(self.x_1)
        self.list2.append(self.y_1)
        self.axes_1.plot(self.list1, self.list2, '*',color='green', linewidth=2)
        self.canvas.draw()
        print type(self.list1),self.list1
        plt.pause(0.001)
        self.axes_1.clear()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window1 = plotnew_1()
    window2 = plotnew_2()
    window1.show()
    window2.show()
    app.exec_()