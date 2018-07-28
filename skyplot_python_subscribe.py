#!/usr/bin/env python
# license removed for brevity
"""
    subscribe environment boundary and visualization
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me
"""
import rospy # ros python needed
from std_msgs.msg import String # standard message needed
import matplotlib as mpl # plotting needed
mpl.use("TkAgg") # Use TKAgg to show figures
import matplotlib.pyplot as plt # plotting needed
import pandas as pd # pandas to pd
import numpy as np # numpy to np
import geometry_msgs.msg as gm # ros geometry message to gm
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # ros message needed
from matplotlib.patches import Ellipse, Circle # draw specific shape
from nlosExclusion.msg import GNSS_Raw_Array,GNSS_Raw # customerized ros message type
from nlosExclusion.msg import Satellite_Info # customized ros message type Satellite_Info containing satellites exclusion numbers
import  math
import csv # csv reading needed library
yawClaibrationAngle2North= -105.57 # define calibrationa angle of car yaw and earth north -110.57
listx_2=[] # double-decker bus boundary is expected to be a line, listx_2 indicates start point in x direction
listy_2=[] # double-decker bus boundary is expected to be a line, listy_2 indicates start point in y direction
listx_3=[] # double-decker bus boundary is expected to be a line, listx_3 indicates end point in x direction
listy_3=[] # double-decker bus boundary is expected to be a line, listy_3 indicates end point in x direction
listx_2_initial=[] # double-decker bus boundary is expected to be a line, listx_2_initial indicates start point in azimuth
listy_2_initial=[] # double-decker bus boundary is expected to be a line, listy_2_initial indicates start point in elevation
listx_3_initial=[] # double-decker bus boundary is expected to be a line, listx_3_initial indicates end point in azimuth
listy_3_initial=[] # double-decker bus boundary is expected to be a line, listy_3_initial indicates end point in elevation
listx_2_ini_glob=[] # double-decker bus boundary is expected to be a line, listx_2_ini_glob indicates start point in azimuth
listx_3_ini_glob=[] # double-decker bus boundary is expected to be a line, listx_3_ini_glob indicates start point in azimuth
listx_2_glob=[] # double-decker bus boundary is expected to be a line, listx_2_glob indicates start point in x direction for NLOS exclusion
listy_2_glob=[] # double-decker bus boundary is expected to be a line, listy_2_glob indicates start point in y direction for NLOS exclusion
listx_3_glob=[] # double-decker bus boundary is expected to be a line, listx_3_glob indicates end point in x direction  for NLOS exclusion
listy_3_glob=[] # double-decker bus boundary is expected to be a line, listy_3_glob indicates end point in x direction for NLOS exclusion
satellite_azimuth_list=[] # create a list to save azimuth for each satellite in one epoch in Skyplot unit
satellite_elevation_list=[] # create a list to save elevation for each satellite in one epoch in Skyplot unit
satellite_azimuth_list_glob=[] # create a list to save azimuth for each satellite in one epoch in Skyplot unit for nlos exclusion
satellite_elevation_list_glob=[] # create a list to save elevation for each satellite in one epoch in Skyplot unit for nlos exclusion
satellite_number_list=[] # create a list to save satellite number (example: G30 number is 30)
satellite_number_list_glob=[] # create a list to save satellite number (example: G30 number is 30) for NLOS exclusion
satellite_initial_azimuth_list=[] # create a list to save satellite initial azimuth
satellite_initial_azimuth_list_glob=[] # create a list to save satellite initial azimuth
satellite_visability=[] # create a list to save satellite visability
satellite_visability_mannually=[14,92,94] # create a list to mannually save satellite visability: for debug purpose 14,27,32,90,92,94
exclusMode=1 # exclusion mode: 0--automatic mdoe  1--mannual mode

GNSS_NLOS_excluded_list =[] #create an list to save GNSS data after NLOS exclusion
plot_total_sate=[] # save total satellites for plotting (GPS+Beidou)
plot_total_sateTim=[] # save time for total satellites for plotting (GPS+Beidou)
plot_GPS_sate=[] # save GPS satellites for plotting
plot_Beidou_sate=[] # save Beidou satellites for plotting
plot_Beidou_sateTim=[] # save time for Beidou satellites for plotting
plot_GPS_sate_excluded=[] # save GPS satellites that have been excluded for plotting
plot_Beidou_sate_excluded=[] # save Beidou satellites that have been excluded for plotting
plot_Beidou_sate_excludedTim=[]
plot_Total_sate_excluded=[] # save total satellites that have been excluded for plotting
plot_Total_sate_excludedTim=[] # save time for total satellites that have been excluded for plotting
GNSS_exclusion_pub = rospy.Publisher('GNSS_exclued_data', GNSS_Raw_Array, queue_size=10)  # customerized GNSS data after NLOS exclusion ros message
Satellite_Info_pub_ = rospy.Publisher('Satellite_Info', Satellite_Info, queue_size=10)  # customerized Satellite Info data after NLOS exclusion ros message


def callback_azi_eleva(data): # callback function to handle azimuth and elevation of raw point cloud
    pose_array_list=[] # create a list to save PoseArray message, which containing azimuth and elevation of raw point cloud
    listx=[] # create a list to save azimuth in degree
    listy=[] # create a list to save elevation in degree
    pose_array_list=data.poses # save all the poses in pose_array_list (one PoseArray message contains several Pose message)
    for i in range(len(pose_array_list)): # index all the Pose message in PoseArray message (http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)
        # x--azimuth y--elevation (Skyplot: https://github.com/feddischson/skyplotwidget)
        if(pose_array_list[i].position.x<0):
            pose_array_list[i].position.x=pose_array_list[i].position.x+360
        listx.append( -1*(pose_array_list[i].position.y*(-0.55556)+50.0)* np.cos(-1*(pose_array_list[i].position.x-90.0+yawClaibrationAngle2North)*3.14159/180.0)) # function to transfer from azimuth into skyplot azimuth
        listy.append( (pose_array_list[i].position.y*(-0.55556)+50.0)* np.sin(-1*(pose_array_list[i].position.x-90.0+yawClaibrationAngle2North)*3.14159/180.0)) # function to transfer from elevation into skyplot elevation
    #draw circle1
    circle_x=circle_y=np.arange(-60, 60, 0.1) # circle parameters
    circle_x,circle_y=np.meshgrid(circle_x,circle_y)
    plt.figure(1)  # create figure 1
    ax1 = plt.subplot(1,1,1)
    # ax2 = plt.subplot(2,2,2)
    # ax3 = plt.subplot(2,1,2)
    plt.sca(ax1)
    plt.contour(circle_x, circle_y, circle_x**2 + circle_y**2, [100])
    plt.contour(circle_x, circle_y, circle_x**2 + circle_y**2, [400])
    plt.contour(circle_x, circle_y, circle_x**2 + circle_y**2, [900])
    plt.contour(circle_x, circle_y, circle_x**2 + circle_y**2, [1600])
    plt.contour(circle_x, circle_y, circle_x**2 + circle_y**2, [2500])
    x_random= np.random.random(50)*20 # random data x
    y_random= np.random.random(50)*20 # random data y
    # plt.plot(x_random, y_random, '*',color='blue')   
    plt.axis("equal") # axis scale mode setting

    #draw circle for skyplot
    circle1=plt.Circle((0,0),50,color='aqua') # circle center at (0,0) with radius 50
    circle2=plt.Circle((0,0),20,color='aqua') # circle center at (0,0) with radius 20
    circle3=plt.Circle((0,0),10,color='aqua') # circle center at (0,0) with radius 10
    plt.gcf().gca().add_artist(circle1) # add circle1
    plt.gcf().gca().add_artist(circle2) # add circle2
    plt.gcf().gca().add_artist(circle3) # add circle3
    #draw circle indicates satellites
    # circle4=plt.Circle((-10,37),4,color='b') # circle center at (-10,37) with radius 4
    # plt.gcf().gca().add_artist(circle4) # add circle4
    # circle5=plt.Circle((-10,-27),4,color='b') # circle center at (-10,-27) with radius 4
    # plt.gcf().gca().add_artist(circle5) # add circle5
    # circle6=plt.Circle((15,-20),4,color='b') # circle center at (-15,-20) with radius 4
    # plt.gcf().gca().add_artist(circle6) # add circle6
    # circle7=plt.Circle((30,-27),4,color='b') # circle center at (30,-27) with radius 4
    # plt.gcf().gca().add_artist(circle7) # add circle7
    satecirclRad=1.0
    for sate_index in range(len(satellite_azimuth_list)): # draw the satellite into the Skyplot (template: circle+G30)
        if( (satellite_number_list[sate_index]>=1) and (satellite_number_list[sate_index]<=32) ): # GPS
            # print 'satellite_visability',satellite_visability
            if( satellite_number_list[sate_index] in satellite_visability): # blockage
                plt.gcf().gca().add_artist( plt.Circle((satellite_azimuth_list[sate_index],satellite_elevation_list[sate_index]),satecirclRad,color='r') )  # add GPS circle
            else: # visable
                plt.gcf().gca().add_artist(plt.Circle((satellite_azimuth_list[sate_index], satellite_elevation_list[sate_index]), satecirclRad,color='g'))  # add GPS circle
            plt.text(satellite_azimuth_list[sate_index], satellite_elevation_list[sate_index], 'G'+str(int(satellite_number_list[sate_index])), fontdict={'size': 15, 'color': 'r'})  # draw the 'G'
            # plt.text(satellite_azimuth_list[sate_index], satellite_elevation_list[sate_index], 'G'+str(int(satellite_initial_azimuth_list[sate_index])), fontdict={'size': 15, 'color': 'r'})  # draw the 'G'
        elif ((satellite_number_list[sate_index] >= 88) and (satellite_number_list[sate_index] <= (87+37))): # Beidou
            if (satellite_number_list[sate_index] in satellite_visability):  # blockage
                plt.gcf().gca().add_artist(plt.Circle((satellite_azimuth_list[sate_index], satellite_elevation_list[sate_index]), satecirclRad,color='r'))  # add Beidou circle
            else:  # visable
                plt.gcf().gca().add_artist(plt.Circle((satellite_azimuth_list[sate_index], satellite_elevation_list[sate_index]), satecirclRad,color='b'))  # add Beidou circle
            plt.text(satellite_azimuth_list[sate_index], satellite_elevation_list[sate_index], 'B'+str(int(satellite_number_list[sate_index])),fontdict={'size': 15, 'color': 'r'})  # draw the 'G'
    print 'satellite_visability=',satellite_visability
    #vertical
    plt.plot([0,50],[0,0],linewidth = '1',color='fuchsia') # draw a line from (0,0) to (50,0)
    plt.text(51, 0, 'E', fontdict={'size': 15, 'color': 'r'}) # draw the 'E'
    plt.plot([0,0],[0,50],linewidth = '1',color='fuchsia') # draw a line from (0,0) to (0,50)
    plt.text(0, 51, 'N', fontdict={'size': 15, 'color': 'r'}) # draw the 'N'
    plt.plot([0,-50],[0,0],linewidth = '1',color='fuchsia') # draw a line from (0,0) to (-50,0)
    plt.text(-54, 0, 'W', fontdict={'size': 15, 'color': 'r'}) # draw the 'W'
    plt.plot([0,0],[0,-50],linewidth = '1',color='fuchsia') # draw a line from (0,0) to (0,-50)
    plt.text(0, -54, 'S', fontdict={'size': 15, 'color': 'r'}) # draw the 'S'
    #tilt and azimuth
    plt.plot([0,43.3],[0,25],linewidth = '1',color='fuchsia')
    plt.text(45, 26, '60', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,25],[0,43.3],linewidth = '1',color='fuchsia')
    plt.text(26, 45, '30', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,-43.3],[0,25],linewidth = '1',color='fuchsia')
    plt.text(-51, 28, '300', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,-25],[0,43.3],linewidth = '1',color='fuchsia')
    plt.text(-27, 48, '330', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,-43.3],[0,-25],linewidth = '1',color='fuchsia')
    plt.text(-51, -30, '240', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,-25],[0,-43.3],linewidth = '1',color='fuchsia')
    plt.text(-29, -48, '210', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,43.3],[0,-25],linewidth = '1',color='fuchsia')
    plt.text(48, -28, '120', fontdict={'size': 15, 'color': 'r'})
    plt.plot([0,25],[0,-43.3],linewidth = '1',color='fuchsia')
    plt.text(27, -47, '150', fontdict={'size': 15, 'color': 'r'})
    #draw elevation indicators
    plt.text(2, 13, '72', fontdict={'size': 15, 'color': 'r'})
    plt.text(2, 23, '54', fontdict={'size': 15, 'color': 'r'})
    plt.text(2, 33, '36', fontdict={'size': 15, 'color': 'r'})
    plt.text(2, 43, '18', fontdict={'size': 15, 'color': 'r'})
    #draw the azimuth and elevation
    plt.plot(listx, listy, '*',color='blue') # listx-(azimuth value in skyplot) listy-(elevation value in skyplot)
    plt.plot(listx_2, listy_2, '*',color='blue') # start point of double-decker bus boundary
    plt.plot(listx_3, listy_3, '*',color='blue') # end point of double-decker bus boundary
    text_falg=0 # initialize variable: make sure that 'double-decker bus' only show one time
    len_listx_2=len(listx_2)
    if(len_listx_2>=3): # set threshold: maxinum double-decker bus for visualization
        len_listx_2=3
    for i_2 in range(len_listx_2):
        print 'len(listx_2)-----------------------',len(listx_2),'len_listx_2',len_listx_2
        plt.plot([listx_2[i_2],listx_3[i_2]],[listy_2[i_2],listy_3[i_2]],linewidth = '2',color='r') # plot a line from (listx_2[i_2],listy_2[i_2]) to (listx_3[i_2],listy_3[i_2])
        plt.plot([0, listx_3[i_2]], [0, listy_3[i_2]], linewidth='2', color='r')
        plt.plot([listx_2[i_2], 0], [listy_2[i_2], 0], linewidth='2', color='r')
        plt.text(listx_2[i_2], listy_2[i_2], str(int(listx_2_initial[i_2])), fontdict={'size': 15, 'color': 'r'})  #
        plt.text(listx_3[i_2], listy_3[i_2], str(int(listx_3_initial[i_2])),fontdict={'size': 15, 'color': 'r'})  #
        # if text_falg==0:
        #     plt.text((listx_2[i_2]+listx_3[i_2])/2, (listy_2[i_2]+listy_3[i_2])/2, 'double-decker bus', fontdict={'size': 15, 'color': 'r'})
        text_falg=1
    for glo_s_0 in range(len(listx_2)):
        listx_2_glob.append(listx_2[glo_s_0])
        listy_2_glob.append(listy_2[glo_s_0])
        listx_3_glob.append(listx_3[glo_s_0])
        listy_3_glob.append(listy_3[glo_s_0])
    del listx_2[:] # relief list context after plot
    del listy_2[:] # relief list context after plot
    del listx_3[:] # relief list context after plot
    del listy_3[:] # relief list context after plot
    for glo_s in range(len(listx_2_initial)): # save the initial azimuth of double-decker bus boundery point for NLOS exclusion
        listx_2_ini_glob.append(listx_2_initial[glo_s]) # boundary point one for NLOS exclusion
        listx_3_ini_glob.append(listx_3_initial[glo_s]) # boundary point two for NLOS exclusion
    del listx_2_initial[:] # relief list context after plot
    del listy_2_initial[:]  # relief list context after plot
    del listx_3_initial[:]  # relief list context after plot
    del listy_3_initial[:]  # relief list context after plot
    del satellite_azimuth_list[:] # relief list context after plot
    del satellite_elevation_list[:] # relief list context after plot
    del satellite_number_list[:] # relief list context after plot
    del satellite_visability[:] # relief list context after plot
    del satellite_initial_azimuth_list[:] # relief list context after plot
    plt.title("skyplot") # specify name of plt
    # plt.sca(ax2) # plot in second part in one figure
    # plt.xlabel('t/s---GPS time', fontdict={'size': 15, 'color': 'r'}) # add label for x coordinate
    # ax2.set_ylim([0,25]) # set range of y coordinate
    # plt.plot(plot_total_sateTim, plot_total_sate, linewidth = '3',color='r', label="satNum") # plot total satellites numbers
    # plt.plot(plot_Beidou_sateTim, plot_GPS_sate, 'g',label="GPSNum") # plot only GPS satellites
    # plt.plot(plot_Beidou_sateTim, plot_Beidou_sate, 'b',label="BeiNum") # plot only Beidou satellites
    # plt.plot(plot_Total_sate_excludedTim, plot_GPS_sate_excluded, 'coral',label="GPSexNum") # plot excluded GPS satellites
    # plt.plot(plot_Total_sate_excludedTim, plot_Beidou_sate_excluded, linewidth = '2',color='darkgreen', label="BeiexNum") # plot excluded Beidou satellites
    # plt.title("satellite numbers")  # specify name of plt
    # ax2.legend(loc=9, ncol=3, shadow=True) # activate the legend
    # plt.sca(ax3) # point to ax3 (after point here, later plot will focus on the ax3)
    # plt.xlabel('t/s---GPS time', fontdict={'size': 15, 'color': 'r'}) # set the label
    # plt.plot(plot_Total_sate_excludedTim, plot_Total_sate_excluded, linewidth='1.5', color='g', label="Total excluded satellites") # plot total excluded satellites
    # ax3.legend(loc=9, ncol=2, shadow=True)  # activate the ax3
    # plt.title("Total excluded satellites ")  # specify name of plt
    plt.draw() # draw plt
    plt.pause(0.00000000001) # pause for a while, if not pause, plt will stop here (a problem of multi-thread)
    plt.clf() # clean the plt


def callback_double_decker_bus_boundary(data):
    pose_array_list_2=[] # create a list to save double-decker bus boundary information
    pose_array_list_2=data.poses # save double-decker bus boundary information
    for i in range(len(pose_array_list_2)):
        #x--azimuth y--elevation
        if(pose_array_list_2[i].orientation.x<0):
            pose_array_list_2[i].orientation.x=pose_array_list_2[i].orientation.x+360
        azi_temp=float(pose_array_list_2[i].orientation.x + yawClaibrationAngle2North)*-1 # angle direction is reverse in skyplot and common coordinate
        if(azi_temp<0):
            azi_temp=azi_temp+360
        listx_2_initial.append(float(azi_temp))  # initial azimuth   start point
        listy_2_initial.append(pose_array_list_2[i].orientation.y)  # initial elevation start point
        if(pose_array_list_2[i].orientation.z<0):
            pose_array_list_2[i].orientation.z=pose_array_list_2[i].orientation.z+360
        azi_temp=float(pose_array_list_2[i].orientation.z + yawClaibrationAngle2North)*-1 # angle direction is reverse in skyplot and common coordinate
        if (azi_temp < 0):
            azi_temp = azi_temp + 360
        listx_3_initial.append(azi_temp)  # initial azimuth   end point
        listy_3_initial.append(pose_array_list_2[i].orientation.w)  # initial elevation end point
        # print 'pose_array_list_2 of double-decker bus boundary',pose_array_list_2
        listx_2.append( -1*(pose_array_list_2[i].orientation.y*(-0.55556)+50.0)* np.cos((pose_array_list_2[i].orientation.x-90.0+yawClaibrationAngle2North)*3.14159/180.0)) # start point azimuth
        listy_2.append( (pose_array_list_2[i].orientation.y*(-0.55556)+50.0)* np.sin(-1*(pose_array_list_2[i].orientation.x-90.0+yawClaibrationAngle2North)*3.14159/180.0)) # start point elevation
        listx_3.append( -1*(pose_array_list_2[i].orientation.w*(-0.55556)+50.0)* np.cos(-1*(pose_array_list_2[i].orientation.z-90.0+yawClaibrationAngle2North)*3.14159/180.0)) # end point azimuth
        listy_3.append( (pose_array_list_2[i].orientation.w*(-0.55556)+50.0)* np.sin(-1*(pose_array_list_2[i].orientation.z-90.0+yawClaibrationAngle2North)*3.14159/180.0)) # end point elevation
        # print 'len(pose_array_list_2)',len(pose_array_list_2)
        # print '-------------'


def callback_GNSS_Raw_data(data): # callback function to handle GNSS Raw data
    GNSS_Raw_array_list=[] # create a list to save PoseArray message, which contains GNSS Raw data
    GNSS_exclusion_Array_1 = GNSS_Raw_Array()  # create an variable in GNSS_Raw_Array format to save GNSS data after exclusion
    GNSS_Raw_array_list=data.GNSS_Raws # save all the satellite azimuth information and elevation information in GNSS_Raw_array_list (one GNSS_Raw_array_list message contains several GNSS_Raws message)
    GNSS_exclusion_Array_1=data
    plot_total_sate.append(GNSS_exclusion_Array_1.GNSS_Raws[0].total_sv) # save the total satellites numbers (GPS+Beidou)
    plot_total_sateTim.append(float(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))
    GPSNum = 0  # create a variable to save GPS number
    BeidouNum = 0  # create a variable to save Beidou number
    for i in range(len(GNSS_Raw_array_list)): # index all the Pose message in PoseArray message (http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)
        # x--azimuth y--elevation (Skyplot: https://github.com/feddischson/skyplotwidget)
        satellite_azimuth_list.append( (GNSS_Raw_array_list[i].elevation*(-0.55556)+50.0)* np.cos(-1*(GNSS_Raw_array_list[i].azimuth-90.0)*3.14159/180.0)) # function to transfer from azimuth into skyplot azimuth
        satellite_elevation_list.append( (GNSS_Raw_array_list[i].elevation*(-0.55556)+50.0)* np.sin(-1*(GNSS_Raw_array_list[i].azimuth-90.0)*3.14159/180.0)) # function to transfer from elevation into skyplot elevation
        satellite_azimuth_list_glob.append((GNSS_Raw_array_list[i].elevation*(-0.55556)+50.0)* np.cos(-1*(GNSS_Raw_array_list[i].azimuth-90.0)*3.14159/180.0))
        satellite_elevation_list_glob.append((GNSS_Raw_array_list[i].elevation*(-0.55556)+50.0)* np.sin(-1*(GNSS_Raw_array_list[i].azimuth-90.0)*3.14159/180.0))
        satellite_number_list.append(GNSS_Raw_array_list[i].prn_satellites_index)
        satellite_number_list_glob.append(GNSS_Raw_array_list[i].prn_satellites_index)
        satellite_initial_azimuth_list.append(GNSS_Raw_array_list[i].azimuth) # save initial azimuth data for plotting in callback_azi_eleva()
        satellite_initial_azimuth_list_glob.append(GNSS_Raw_array_list[i].azimuth) # save initial azimuth data for NLOS exclusion in callback_GNSS_Raw_data
        if ((GNSS_Raw_array_list[i].prn_satellites_index >= 1) and (GNSS_Raw_array_list[i].prn_satellites_index <= 32)):  # GPS
            GPSNum=GPSNum+1
        if ((GNSS_Raw_array_list[i].prn_satellites_index >= 88) and (GNSS_Raw_array_list[i].prn_satellites_index <= (87+37))):  # Beidou
            BeidouNum=BeidouNum+1
    if (GPSNum > 0):  # make sure that GPSNum is not zero
        plot_GPS_sate.append(float(GPSNum))  # save GPS number to list
        plot_Beidou_sate.append(float(BeidouNum))  # save Beidou number to list
        plot_Beidou_sateTim.append(float(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))
    GPSNum = 0  # clear
    BeidouNum = 0  # clear
    #NLOS Exclusion
    len_listx_2 = len(listx_2_ini_glob)
    if (len_listx_2 >= 3): # set threshold: maxinum double-decker bus for NLOS exclusion
        len_listx_2 = 3
    if(exclusMode==0): # autonomatic exclusion
        for i_bou_num in range(len_listx_2): # index all the boundary line (usually only one double-decker bus, sometime 2)
            for index_sat in range(len(GNSS_Raw_array_list)):  # index all the satellite information in one epoch
                point0_x = 0.0  # yuanxin x
                point0_y = 0.0  # yuanxin y
                point1_x=float(listx_2_glob[i_bou_num]) # point1 coordinate x
                point1_y=float(listy_2_glob[i_bou_num]) # point1 coordinate y
                point2_x=float(listx_3_glob[i_bou_num]) # point2 coordinate x
                point2_y=float(listy_3_glob[i_bou_num]) # point2 coordinate y
                point_x =float(satellite_azimuth_list_glob[index_sat]) # point_x (satellite azimuth in skyplot)
                point_y =float(satellite_elevation_list_glob[index_sat]) # point_y (satellite elevation in skyplot)
                area_error=math.fabs(IsInside(point1_x, point1_y, point2_x, point2_y, point0_x, point0_y, point_x, point_y))
                # print 'area_error',area_error,'listx_2_ini_glob',listx_2_ini_glob
                # print 'len(listx_2)=',len(listx_2),'len(listx_2_ini_glob)=',len(listx_2_ini_glob)
                if(listx_2_ini_glob[i_bou_num]<0): # negtive to positive
                    listx_2_ini_glob[i_bou_num]=listx_2_ini_glob[i_bou_num]+360
                if (listx_3_ini_glob[i_bou_num] < 0): # negtive to positive
                    listx_3_ini_glob[i_bou_num] = listx_3_ini_glob[i_bou_num] + 360
                if(listx_2_ini_glob[i_bou_num]>listx_3_ini_glob[i_bou_num]) and len(listx_2_ini_glob)>0: # make sure that listx_3_initial[i_bou_num] is smaller
                    azi_temp=listx_3_ini_glob[i_bou_num]
                    listx_3_ini_glob[i_bou_num]=listx_2_ini_glob[i_bou_num]
                    listx_2_ini_glob[i_bou_num]=azi_temp
                ang_ran_=float(listx_3_ini_glob[i_bou_num]-listx_2_ini_glob[i_bou_num])
                print 'ang_ran_----------------------',ang_ran_,'from',listx_2_ini_glob[i_bou_num],'to',listx_3_ini_glob[i_bou_num]
                # if(len(satellite_visability)>0): # if satellite_visability is not empty, clear
                #     del satellite_visability[:] # clear content of satellite_visability
                if(ang_ran_<160):
                    if(satellite_initial_azimuth_list_glob[index_sat]>listx_2_ini_glob[i_bou_num]) and (satellite_initial_azimuth_list_glob[index_sat]<listx_3_ini_glob[i_bou_num]) and (area_error>5):# and (math.fabs(area_error)>3) :
                        print 'detect one NLOS signals at satellite-',satellite_number_list_glob[index_sat],'area_error',area_error,'satellite_initial_azimuth_list',satellite_initial_azimuth_list_glob[index_sat]
                        print 'point1_x point1_y point2_x point2_y point_x point_y',point1_x,point1_y,point2_x,point2_y,point_x,point_y
                        GNSS_exclusion_Array_1.GNSS_Raws[index_sat].visable = 2 # for demonstration: this element will be deleted later
                        satellite_visability.append(float(GNSS_exclusion_Array_1.GNSS_Raws[index_sat].prn_satellites_index)) # save the visability
                elif(ang_ran_>160):
                    if (satellite_initial_azimuth_list_glob[index_sat] > 0) and (satellite_initial_azimuth_list_glob[index_sat] < listx_2_ini_glob[i_bou_num]) and (area_error >5):  # and (math.fabs(area_error)>3) :
                        print 'detect one NLOS signals at satellite-', satellite_number_list_glob[index_sat], 'area_error', area_error, 'satellite_initial_azimuth_list',satellite_initial_azimuth_list_glob[index_sat]
                        print 'point1_x point1_y point2_x point2_y point_x point_y', point1_x, point1_y, point2_x, point2_y, point_x, point_y
                        GNSS_exclusion_Array_1.GNSS_Raws[index_sat].visable = 2 # for demonstration: this element will be deleted later
                        satellite_visability.append(float(GNSS_exclusion_Array_1.GNSS_Raws[index_sat].prn_satellites_index)) # save the visability
                    if (satellite_initial_azimuth_list_glob[index_sat] < 360) and (satellite_initial_azimuth_list_glob[index_sat] > listx_3_ini_glob[i_bou_num]) and (area_error >5):  # and (math.fabs(area_error)>3) :
                        print 'detect one NLOS signals at satellite-', satellite_number_list_glob[index_sat], 'area_error', area_error, 'satellite_initial_azimuth_list',satellite_initial_azimuth_list_glob[index_sat]
                        print 'point1_x point1_y point2_x point2_y point_x point_y', point1_x, point1_y, point2_x, point2_y, point_x, point_y
                        GNSS_exclusion_Array_1.GNSS_Raws[index_sat].visable = 2 # for demonstration: this element will be deleted later
                        satellite_visability.append(float(GNSS_exclusion_Array_1.GNSS_Raws[index_sat].prn_satellites_index)) # save the visability
        GPSNumExcluded=0.0
        BeidouNumExcluded=0.0
        for del_index in range(len(GNSS_exclusion_Array_1.GNSS_Raws)): # index all the satellites' visability in one epoch
            if (del_index >= len(GNSS_exclusion_Array_1.GNSS_Raws)):
                break
            if(GNSS_exclusion_Array_1.GNSS_Raws[del_index].visable == 2) or(GNSS_exclusion_Array_1.GNSS_Raws[del_index].elevation<=20): # if satellite is NLOS reception, delete this satellite
                if ((GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index >= 1) and (GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index <= 32)):  # GPS
                    GPSNumExcluded=GPSNumExcluded+1 # one GPS exclusion
                if ((GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index >= 88) and (GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index <= (87+37))):  # GPS
                    BeidouNumExcluded=BeidouNumExcluded+1 # one Beidou exclusion
                del GNSS_exclusion_Array_1.GNSS_Raws[del_index]  # delete the NLOS reception satellites
    if(exclusMode==1): # mannual exclusion: specify certain amount of satellites needed to be excluded
        for index_sat in range(len(GNSS_Raw_array_list)):  # index all the satellite information in one epoch
                if(GNSS_exclusion_Array_1.GNSS_Raws[index_sat].prn_satellites_index in satellite_visability_mannually):# and (math.fabs(area_error)>3) :
                    GNSS_exclusion_Array_1.GNSS_Raws[index_sat].visable = 2 # for demonstration: this element will be deleted later
                    if float(GNSS_exclusion_Array_1.GNSS_Raws[index_sat].prn_satellites_index) not in satellite_visability:
                        satellite_visability.append(float(GNSS_exclusion_Array_1.GNSS_Raws[index_sat].prn_satellites_index)) # save the visability
        GPSNumExcluded=0.0
        BeidouNumExcluded=0.0
        for del_index in range(len(GNSS_exclusion_Array_1.GNSS_Raws)): # index all the satellites' visability in one epoch
            if (del_index >= len(GNSS_exclusion_Array_1.GNSS_Raws)):
                break
            if(GNSS_exclusion_Array_1.GNSS_Raws[del_index].visable == 2) or(GNSS_exclusion_Array_1.GNSS_Raws[del_index].elevation<=20): # if satellite is NLOS reception, delete this satellite
                if ((GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index >= 1) and (GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index <= 32)):  # GPS
                    GPSNumExcluded=GPSNumExcluded+1 # one GPS exclusion
                if ((GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index >= 88) and (GNSS_exclusion_Array_1.GNSS_Raws[del_index].prn_satellites_index <= (87+37))):  # GPS
                    BeidouNumExcluded=BeidouNumExcluded+1 # one Beidou exclusion
                    print 'GNSS_exclusion_Array_1.GNSS_Raws[del_index].elevation',GNSS_exclusion_Array_1.GNSS_Raws[del_index].elevation
                del GNSS_exclusion_Array_1.GNSS_Raws[del_index]  # delete the NLOS reception satellites

    plot_GPS_sate_excluded.append(float(GPSNumExcluded)) # save GPS exclusion numbers
    plot_Beidou_sate_excluded.append(float(BeidouNumExcluded)) # save Beidou exclusion numbers
    plot_Total_sate_excluded.append(float(GPSNumExcluded)+float(BeidouNumExcluded)) # save all the satellites that have been excluded
    plot_Total_sate_excludedTim.append(float(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))
    satelliteInfo=Satellite_Info()
    satelliteInfo.GNSS_time      = int(float(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))
    satelliteInfo.GPSExcluded    = float(GPSNumExcluded)
    satelliteInfo.BeidouExcluded = float(BeidouNumExcluded)
    Satellite_Info_pub_.publish(satelliteInfo)
    GPSNumExcluded = 0.0 # clear numbers
    BeidouNumExcluded = 0.0 # clear numbers
    if(len(listx_2_ini_glob)>0): # make sure, publish GNSS_exclusion_Array_1 only when there is at least one double-decker bus
        GNSS_exclusion_pub.publish(GNSS_exclusion_Array_1)
    del listx_2_ini_glob[:]
    del listx_3_ini_glob[:]
    del listx_2_glob[:]
    del listy_2_glob[:]
    del listx_3_glob[:]
    del listy_3_glob[:]
    del satellite_initial_azimuth_list_glob[:]
    del satellite_azimuth_list_glob[:]
    del satellite_elevation_list_glob[:]
    del satellite_number_list_glob[:]


def IsTrangleOrArea(x1, y1, x2, y2, x3, y3):
    return math.fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)


def IsInside(x1, y1, x2, y2, x3, y3, x, y):
    # rectangle ABC area
    ABC = IsTrangleOrArea(x1, y1, x2, y2, x3, y3)

    # rectangle PBC area
    PBC = IsTrangleOrArea(x, y, x2, y2, x3, y3)

    # rectangle ABC area
    PAC = IsTrangleOrArea(x1, y1, x, y, x3, y3)

    # rectangle ABCarea
    PAB = IsTrangleOrArea(x1, y1, x2, y2, x, y)
    Area_error_temp= float(ABC-(PBC + PAC + PAB))
    return float(ABC-(PBC + PAC + PAB))


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('skyplot_python_subscribe', anonymous=True) # initialize node and specify node name
    rospy.Subscriber("azi_eleva", PoseArray, callback_azi_eleva) # initialize subscriber to "azi_eleva", which contains azimuth and elevetion of raw point cloud
    rospy.Subscriber('double_decker_parameters', PoseArray, callback_double_decker_bus_boundary) # initialize subscriber to "double_decker_parameters", which contains azimuth and elevetion of double-decker bus boundary
    rospy.Subscriber('GNSS_RAW', GNSS_Raw_Array, callback_GNSS_Raw_data)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello_str = "skyplot_python_subscribe node running at time %s" % rospy.get_time()
        rate.sleep()


if __name__ == '__main__':
    listener()
