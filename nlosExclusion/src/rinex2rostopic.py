#!/usr/bin/env python
# license removed for brevity
"""
    Reads customized RINEX2 or RINEX3 file into ros topic
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me
"""
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
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs
from pointcloud_to_laserscan.msg import GNSS_Raw_Array,GNSS_Raw # customerized ros message type
from matplotlib.patches import Ellipse, Circle # draw circle needs library
import csv # csv reading needed library
import datetime #time format (datetime)
import time #time format (time)
import llh2ecef # llh to ecef
import skyplot_python_subscribe
from pointcloud_to_laserscan.msg import Satellite_Info # customized ros message type Satellite_Info containing satellites exclusion numbers

#read csv file into rostopic (GNSS Raw data)
csv_reader=csv.reader(open('/home/wenws/23_pointcloud2laserscan/src/pointcloud_to_laserscan/src/data5/exper1/sv_data.csv', 'r')) #read csv context to csv_reader variable
filename='/home/wenws/23_pointcloud2laserscan/src/pointcloud_to_laserscan/src/data5/exper1/llhWitNLOSExlusion.csv' # positioning result with NLOS exclusion
filename_2='/home/wenws/23_pointcloud2laserscan/src/pointcloud_to_laserscan/src/data5/exper1/llhRaw.csv' # positioning result without NLOS exclusion
list_lat=[] #create a list to save latitude from recalculated GNSS positioning (with NLOS exclusion)
list_lon=[] #create a list to save longitude from recalculated GNSS positioning (with NLOS exclusion)
listLatWiNLOSExc=[] # save Lat to a list for scv file saving after NLOS exclusion
listLonWiNLOSExc=[] # save Lon to a list for scv file saving after NLOS exclusion
listLatRaw=[]  # save Raw Lat to a list for scv file saving
listLonRaw=[]  # save Raw Lon to a list for scv file saving
csv_readerGt=csv.reader(open('/home/wenws/23_pointcloud2laserscan/src/pointcloud_to_laserscan/src/groundTruth.csv', 'r')) #read csv context to csv_reader variable
plot_RGNSSyGtBias=[] # save bias between Raw GNSS positioning result and Ground truth
plot_ExclGNSSyGtBias=[] # save bias between excluded GNSS positioning result and Ground truth
plot_ExclGNSSyGtBiasTim=[] # save time for bias between excluded GNSS positioning result and Ground truth
plot_Dop_value=[] # save dillution of precision value
latDic={} # dictionary to save latitude ground truth
lonDic={} # dictionary to save longitude ground truth
latTruth=[] # list to save lat ground truth
lonTruth=[] # list to save lon ground truth
plot_RawGNSSlat=[] # save raw GNSS latitude for plotting
plot_RawGNSSlon=[] # save raw GNSS longitude for plotting
plot_ExclLat=[] # save the excluded lat for plotting
plot_ExclLon=[] # save the exclude lon for plotting
plot_RawHdop=[]    # save the Raw Hdop for plotting
plot_RawHdopTim=[] # save time for the Raw Hdop for plotting
plot_excludedHdop=[] # save the excluded Hdop for plotting
plot_excludedHdopTim=[] # save time for the excluded Hdop for plotting
plot_visiExclu=[] # save visibility of excluded satellites
plot_SNR      =[] # save SNR of excluded satellites
plot_stanDeviation=[] # save satndard deviation

GNSSTimeList=[] # list of GNSS time
plot_Beidou_sate_excluded_    =[] # save number the satellite of excluded Beidou
plot_GPS_sate_excluded_       =[] # save number the satellite of excluded GPS
plot_Beidou_sate_excluded_Tim =[] # save time for satellites exclusion time plot
plot_Total_sate               =[] # save the Total satellites
plot_Beidou_sate              =[] # save the Beidou satellites
plot_GPS_sate                 =[] # save the GPS satellites
plot_Total_sate_Tim           =[] # save the tinme for total satellites


for csv_row in csv_readerGt: # get ground truth
    latDic[str(csv_row[0])] = str(csv_row[1]) # get latitude ground truth to Dic
    lonDic[str(csv_row[0])] = str(csv_row[2]) # get longitude ground truth to Dic
    latTruth.append(float(csv_row[1])) # get latitude ground truth to list
    lonTruth.append(float(csv_row[2])) # get longitude ground truth to list


def csv2topic():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('csv2topic', anonymous=True) #initialize node with specific node name
    GNSS_Raw_pub = rospy.Publisher('GNSS_RAW', GNSS_Raw_Array, queue_size=10) #customerized GNSS raw data ros message
    Navsatfix_pub= rospy.Publisher('navsatfix_llh',NavSatFix,  queue_size=10) #standard GNSSs meaasge with lat and longt
    rospy.Subscriber('GNSS_exclued_data', GNSS_Raw_Array, callback_GNSS_data_after_NLOS_exclusion)
    rospy.Subscriber('Satellite_Info', Satellite_Info, callback_Satellite_NLOS_exclusion)

    GNSS_Raw_Array_1 = GNSS_Raw_Array() #create an variable in GNSS_Raw_Array format
    GNSS_options=1 # 0-GPS only  1-hybrid GNSS (GPS+Beidou)
    sate_index_range=330 # GPS only (sate_index_range=33)  Hybrid GNSS positioning (sate_index_range=330)
    line_index=0 # variable represents line index in csv file
    gnss_time=0 # GNSS time contained in csv file for each epoch (epoch: each time point)
    previous_gnss_time=0.0 # previous GNSS time contained in csv file for each epoch (epoch: each time point)
    previoustime=time.time() #initialize variable
    currenttime=time.time() #initialize variable
    plot_time = 0.0
    #get data from .csv file--------------------begin----------------------------
    for row_csv in csv_reader: #index all rows in csv file
        plt.figure(1)  # create figure 1
        ax1 = plt.subplot(3, 1, 1)
        ax2 = plt.subplot(3, 1, 2)
        ax3 = plt.subplot(3, 1, 3)
        gnss_time=row_csv[0] #get first data in each row (GNSS time exactly)
        GNSS_Raw_1 = GNSS_Raw() # create customerized variable in GNSS_Raw format (element for GNSS_Raw_Array)
        NavSatFix_llh = NavSatFix() # creat standard variable in NavSatFix format
        # print '--------------begin one row----------'
        # print row_csv[12],len(row_csv),type(row_csv)
        # data_test=float(row_csv[3])
        GNSS_Raw_1.GNSS_time=           float(row_csv[0]) # GNSS time contained in csv file
        GNSS_Raw_1.total_sv=            float(row_csv[1]) # total sitellites in one epoch (epoch: each time point)
        GNSS_Raw_1.prn_satellites_index=float(row_csv[2]) # satellite index in this epoch (epoch: each time point)
        GNSS_Raw_1.pseudorange=         float(row_csv[3])-float(row_csv[7])-float(row_csv[8])+float(row_csv[9]) # pseudorange measurement
        GNSS_Raw_1.snr=                 float(row_csv[4]) # signal noise ratio for this satellite
        GNSS_Raw_1.elevation=           float(row_csv[5]) # elevation for this satellite and receiver
        GNSS_Raw_1.azimuth=             float(row_csv[6]) # azimuth for this satellite
        GNSS_Raw_1.err_tropo=           float(row_csv[7]) # troposphere error in meters
        GNSS_Raw_1.err_iono=            float(row_csv[8]) # ionophere error in meters
        GNSS_Raw_1.sat_clk_err=         float(row_csv[9]) # satellite clock bias caused error
        GNSS_Raw_1.sat_pos_x=           float(row_csv[10]) # satellite positioning in x direction
        GNSS_Raw_1.sat_pos_y=           float(row_csv[11]) # satellite positioning in y direction
        GNSS_Raw_1.sat_pos_z=           float(row_csv[12]) # satellite positioning in Z direction
        GNSS_Raw_1.visable=             0                  # satellite visability juded by NLOS exclusion
        if((previous_gnss_time==gnss_time) or (line_index==0)): # differentiate each epoch
            if(GNSS_Raw_1.prn_satellites_index<sate_index_range): ## GPS only (sate_index_range=33)  Hybrid GNSS positioning (sate_index_range=330)
                GNSS_Raw_Array_1.GNSS_Raws.append(GNSS_Raw_1) #save data for each epoch
        else:
            GNSS_Raw_Array_1.header.frame_id='GNSS_Raw_' # set frame_id
            # for del_index in range(len(GNSS_Raw_Array_1.GNSS_Raws)):  # index all the satellites' visability in one epoch
            #     if (del_index >= len(GNSS_Raw_Array_1.GNSS_Raws)):
            #         break
            #     if (GNSS_Raw_Array_1.GNSS_Raws[del_index].elevation <= 20):  # if satellite is low elevation satellite, delete this satellite
            #         del GNSS_Raw_Array_1.GNSS_Raws[del_index]  # delete the low elevation satellites
            GNSS_Raw_pub.publish(GNSS_Raw_Array_1) # publish GNSS_Raw_Array topic after one epoch
            plot_time=plot_time+1
            plot_RawHdop.append(float(DopCalculation(GNSS_Raw_Array_1)))  # calculate dop matrix
            plot_RawHdopTim.append(float(GNSS_Raw_1.GNSS_time))
            GPS_satellite_number=[]
            Beidou_satellite_number=[]
            GPS_satellite_number,Beidou_satellite_number=get_GPS_Beidou_satellite_numbers(GNSS_Raw_Array_1)
            plot_Total_sate.append(len(GPS_satellite_number)+len(Beidou_satellite_number))  # save the Total satellites
            plot_Beidou_sate.append(len(Beidou_satellite_number))  # save the Beidou satellites
            plot_GPS_sate.append(len(GPS_satellite_number))        # save the GPS satellites
            plot_Total_sate_Tim.append(float(GNSS_Raw_1.GNSS_time))  # save the tinme for total satellites
            print ' one epoch...... ','BeiN',len(GPS_satellite_number),'GPSN',len(Beidou_satellite_number)  # indicates one epoch (epoch: each time point)
            iterations=0.0 # initialize iterations times
            itera_x = 0.0  # initialize receiver's position x
            itera_y = 0.0  # initialize receiver's position y
            itera_z = 0.0  # initialize receiver's position z
            itera_b = 0.0  # initialize receiver's clock bias error of GPS only positioning in meters
            itera_bias_=0.0 # initialize total bias of GPS only positioning in each iteration
            itera_b_GPS=0.0 # initialize receiver's clock bias error of GPS in Hybrid GNSS positioning in meters
            itera_b_Beidou=0.0 # initialize receiver's clock bias error of Beidou in Hybrid GNSS positioning in meters
            if(len(GNSS_Raw_Array_1.GNSS_Raws)>3) and (GNSS_options==0): #at least four satellites for least square calculation with GPS only
                itera_x, itera_y, itera_z, itera_b, itera_bias_ = weighted_lesat_square_GNSS_positioning(GNSS_Raw_Array_1, 0.1, 0.1, 0.1, 1.0) # first iteration from (0.1, 0.1, 0.1, 1.0)
                while (itera_bias_>1e-4) and iterations<10: #threshold for iterations:value and times
                    itera_x, itera_y, itera_z ,itera_b,itera_bias_= weighted_lesat_square_GNSS_positioning(GNSS_Raw_Array_1, itera_x, itera_y, itera_z,itera_b) # iteration
                    iterations=iterations+1 # add one iteration
            if (len(GNSS_Raw_Array_1.GNSS_Raws) > 4) and (len(Beidou_satellite_number)>=1) and (GNSS_options==1):  # at least five satellites (at least one Beidou) for least square calculation with Hybrid GNSS
                itera_x, itera_y, itera_z, itera_b_GPS,itera_b_Beidou,itera_bias_ = weighted_lesat_square_GNSS_positioning_hybrid_GNSS(GNSS_Raw_Array_1, 0.1, 0.1, 0.1, 1.0, 1.0)  # first iteration from (0.1, 0.1, 0.1, 1.0)
                while (itera_bias_ > 1e-4) and iterations < 10:  # threshold for iterations:value and times
                    itera_x, itera_y, itera_z, itera_b_GPS, itera_b_Beidou,itera_bias_ = weighted_lesat_square_GNSS_positioning_hybrid_GNSS(GNSS_Raw_Array_1, itera_x, itera_y, itera_z, itera_b_GPS,itera_b_Beidou)  # iteration
                    iterations = iterations + 1  # add one iteration
            print 'iterations=', iterations,'itera_bias_=', itera_bias_ # how many iterations after sucessfully least square calculation, total bioas
            print 'skyplot_python_subscribe.plot_total_sate',skyplot_python_subscribe.plot_total_sate
            iterations=0.0 #initialize iterations variable
            #get llh information
            if(itera_x!=0): # make sure that itera_x is not zero value, otherwise arctan(itera_y/itera_x) in xyz2llh function
                itera_xyz=[] # create a list to save ecef coordiante
                itera_xyz.append(float(itera_x)) # save x value
                itera_xyz.append(float(itera_y)) # save y value
                itera_xyz.append(float(itera_z)) # save z value
                llh_output=ecef2llh.xyz2llh(itera_xyz) # ecef to llh coordinates
                if(latDic.has_key(str(int(gnss_time)))):
                    truthList=[] # create a list to save ground truth
                    truthList.append(float(latDic[str(int(gnss_time))])) # get truth lat
                    truthList.append(float(lonDic[str(int(gnss_time))])) # get truth lon
                    xyzTruth=llh2ecef.llh2xyz(truthList) # get xyz from ecef
                    delt_x = (xyzTruth[0] - itera_xyz[0]) * (xyzTruth[0] - itera_xyz[0]) # get delta x (Error in x)
                    delt_y = (xyzTruth[1] - itera_xyz[1]) * (xyzTruth[1] - itera_xyz[1]) # get delta y (Error in y)
                    delt_z = (xyzTruth[2] - itera_xyz[2]) * (xyzTruth[2] - itera_xyz[2]) # get delta z (Error in z)
                    delErrTem=sqrt(delt_x+delt_y+delt_z)
                    if(delErrTem>100):
                        delErrTem=100
                    plot_RGNSSyGtBias.append(delErrTem) # save error
                    GNSSTimeList.append(float(GNSS_Raw_1.GNSS_time))
                    # print 'xyzTruth=',xyzTruth
                # print 'llh output=',llh_output,'line_index=',line_index,'itera_xyz=',itera_xyz # llh coordinates and line index
                NavSatFix_llh.latitude = float(llh_output[0]) # save latitude to NavSatFix_llh message
                NavSatFix_llh.longitude = float(llh_output[1]) # save longitude to NavSatFix_llh message
                plot_RawGNSSlat.append(float(llh_output[0])) # save raw GNSS latitude for plotting
                plot_RawGNSSlon.append(float(llh_output[1])) # save raw GNSS longitude for plotting
                Navsatfix_pub.publish(NavSatFix_llh) #publish NavSatFix_llh message
                #route of vehicles in matplotlib
                plt.sca(ax1)
                # plt.plot(lonTruth,latTruth, 'o', color='r',label="ground truth")  # plot discrete position of receiver (Ground Truth)
                # plt.plot(plot_RawGNSSlon , plot_RawGNSSlat,'*', color='g',label="GNSS positioning") # plot discrete position of receiver
                listLatRaw.append(float(llh_output[0])) # save Raw Lat
                listLonRaw.append(float(llh_output[1])) # save Raw lon
            if(len(list_lat)>0):
                plot_ExclLat.append(float(list_lon[0]))
                plot_ExclLon.append(float(list_lat[0]))
                # plt.plot(list_lon,list_lat, '*', color='g')
                del list_lat[:]
                del list_lon[:]
            plt.plot(plot_Beidou_sate_excluded_Tim, plot_Beidou_sate_excluded_, '*-', color='r',label="Excluded Beidou numbers")  # plot discrete position of receiver
            plt.plot(plot_Beidou_sate_excluded_Tim, plot_GPS_sate_excluded_, '*-', color='g',label="Excluded GPS Numbers")  # plot discrete position of receiver
            plt.plot(plot_Total_sate_Tim, plot_Beidou_sate, '*-',linewidth='2',color='b',label="total Beid Numbers")  # plot discrete position of receiver
            plt.plot(plot_Total_sate_Tim, plot_GPS_sate, '*-',linewidth='2', color='fuchsia',label="total GPS Numbers")  # plot discrete position of receiver

            ax1.legend(loc=10, ncol=3, shadow=True)
            # plt.title('satellite exclusion numbers') # set title
            plt.sca(ax2) # plot in another sca
            plt.plot(GNSSTimeList, plot_RGNSSyGtBias,'*-',color= 'g', label="RawGNSSerr") # raw GNSS positioning error
            plt.plot(plot_ExclGNSSyGtBiasTim, plot_ExclGNSSyGtBias, '*-',linewidth='4', color='r', label="ExclGNSSerr") # excluded GNSS positioning error
            ax2.legend(loc=9, ncol=3, shadow=True)
            plt.sca(ax3)  # plot in another sca
            plt.xlabel('t/s---GPS time', fontdict={'size': 15, 'color': 'r'})
            plt.plot(plot_RawHdopTim, plot_RawHdop,  '*-',linewidth='2', color='g', label="HDOP error") # Raw GNSS Hdop error
            plt.plot(plot_excludedHdopTim, plot_excludedHdop, '*-',linewidth='4', color='r', label="Excluded HDOP error")  # excluded GNSS Hdop error
            ax3.legend(loc=9, ncol=3, shadow=True)
            plt.draw() # draw figure
            plt.pause(0.00000000001) # pause for a while to immitate multi-thread
            time.sleep(0.4)        # control frequency of topic publishment
            plt.clf()  # clean the plt

            del GNSS_Raw_Array_1.GNSS_Raws[:]#relief the content
            if (GNSS_Raw_1.prn_satellites_index < sate_index_range): # # GPS only (sate_index_range=33)  Hybrid GNSS positioning (sate_index_range=330)
                GNSS_Raw_Array_1.GNSS_Raws.append(GNSS_Raw_1) # save GNSS_Raws to GNSS_Raw_Array_1 variable
        previous_gnss_time=gnss_time # save time to previous time
        line_index=line_index+1 # add one line index
    with open(filename, 'w') as file_object: # save listLatRaw and listLonRaw to csv file
        for sav_idx in range(len(listLatWiNLOSExc)):
            str2=str(listLatWiNLOSExc[sav_idx])+','+str(listLonWiNLOSExc[sav_idx])
            file_object.write(str2)
            file_object.write('\n')
            # print 'str1 in one line....',str2
    # print 'finish saving llhWitNLOSExlu.csv ... with ',listLatWiNLOSExc,'items'
    with open(filename_2, 'w') as file_object_2:  # save listLatRaw and listLonRaw to csv file
        for sav_idx_2 in range(len(listLatRaw)):
            str1 = str(listLatRaw[sav_idx_2]) + ',' + str(listLonRaw[sav_idx_2])
            file_object_2.write(str1)
            file_object_2.write('\n')
    # plt.draw()
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello_str = "current time is %s" % rospy.get_time()
        rate.sleep()
    """
    input:GNSS_Raw_Array_1  --one epoch of GNSS Raw data 
    float64 GNSS_time
    float64 total_sv
    float64 prn_satellites_index
    float64 pseudorange
    float64 snr
    float64 elevation
    float64 azimuth
    float64 err_tropo
    float64 err_iono
    float64 sat_clk_err
    float64 sat_pos_x
    float64 sat_pos_y
    float64 sat_pos_z
    int64 visable #0-Not sure 1-visable 2-invisable
    output:Receiver positioning information
    Expected ENU output:latitude22.3033 longitude114.179472
    """

def callback_Satellite_NLOS_exclusion(data_sat): # save satellites information
    satelliteInfo_=Satellite_Info()
    satelliteInfo_=data_sat
    plot_Beidou_sate_excluded_.append(satelliteInfo_.BeidouExcluded)  # save number the satellite of excluded Beidou
    plot_GPS_sate_excluded_.append(satelliteInfo_.GPSExcluded)  # save number the satellite of excluded GPS
    plot_Beidou_sate_excluded_Tim.append(satelliteInfo_.GNSS_time)  # save time for satellites exclusion time plot
    print 'one call back in satellite info ',len(plot_GPS_sate_excluded_),len(plot_GPS_sate_excluded_),len(plot_Beidou_sate_excluded_Tim)


def weighted_lesat_square_GNSS_positioning(GNSS_one_epoch,init_x,init_y,init_z,init_b): # least square
    # -------------------------------------------------initialization
    rec_ini_pos_x=float(init_x) #initial position x (m)
    rec_ini_pos_y=float(init_y) #initial position y (m)
    rec_ini_pos_z=float(init_z) #initial position z (m)
    rec_clo_bia_b=float(init_b) #initial distance bias caused by clock bias (m)
    devia_xyz    =1.0           #initial set receiver position estimation error (m)
    for coun in range(1):
    #     print '.....................................begin one iteration.................',len(GNSS_one_epoch.GNSS_Raws)
        gues_pseud=[] #guessed pseudorange
        pseud_error=[] #pseudorange error
        G_matrix_  =array([2,2,2,1],dtype='float') #creat G matrix to save transform parameters
        #get guessed pseudorange and pseudorange error
        for index_1 in range(len(GNSS_one_epoch.GNSS_Raws)): # index all the satellite information in one epoch
            sx_1=float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_x) # satellite position x
            sy_1=float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_y) # satellite position y
            sz_1=float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_z) # satellite position z
            sx_1 = (sx_1 - rec_ini_pos_x) * (sx_1 - rec_ini_pos_x)  # satellite to receiver distance in x idrection
            sy_1 = (sy_1 - rec_ini_pos_y) * (sy_1 - rec_ini_pos_y)  # satellite to receiver distance in y idrection
            sz_1 = (sz_1 - rec_ini_pos_z) * (sz_1 - rec_ini_pos_z)  # satellite to receiver distance in z idrection
            sat2rec_dis=sqrt(sx_1 + sy_1 + sz_1) # guessed pseudorange
            gues_pseud.append(sat2rec_dis) # save guessed pseudorange
            pseud_error_element=float(GNSS_one_epoch.GNSS_Raws[index_1].pseudorange) - float(gues_pseud[index_1]) + float(rec_clo_bia_b) # pseudorange error
            pseud_error.append(pseud_error_element) # save pseudorange error
            G_row = []  # G matrix row
            G_row.append(float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_x - rec_ini_pos_x) / float (gues_pseud[index_1]) * -1) # x for G matrix row
            G_row.append(float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_y - rec_ini_pos_y) / float (gues_pseud[index_1]) * -1) # y for G matrix row
            G_row.append(float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_z - rec_ini_pos_z) / float (gues_pseud[index_1]) * -1) # z for G matrix row
            element_float=1.0 # last element for each row
            G_row.append(element_float) # save last element for each row
            G_matrix_=np.row_stack((G_matrix_, G_row)) # add each row to G_matrix
            del G_row[:] # relief G_row
        #get pseudorange error
        pseud_error_mat=np.array(pseud_error) #from list to array
        pseud_error_mat=pseud_error_mat.transpose() # transpose
        #get G matrix
        G_matrix_=np.delete(G_matrix_, [0], axis=0) # delete the first row of G matrix

        delta_p=np.dot((G_matrix_.transpose()),G_matrix_) # G(T) * G
        delta_p_2=np.linalg.inv(delta_p) # inverse matrix of G(T) * G
        delta_p=np.dot(delta_p_2,(G_matrix_.transpose())) # multiply (inverse matrix of G(T) * G) and G(T)
        delta_p=np.dot(delta_p,pseud_error_mat) # multiply with pseud_error_mat
        rec_ini_pos_x = rec_ini_pos_x + float(delta_p[0]) # update receiver position in x direction
        rec_ini_pos_y = rec_ini_pos_y + float(delta_p[1]) # update receiver position in y idrection
        rec_ini_pos_z = rec_ini_pos_z + float(delta_p[2]) # update receiver position in z idrection
        rec_clo_bia_b = rec_clo_bia_b + float(delta_p[3]) # update receiver clock bias in meters
        devia_x = float(delta_p[0]) # save delta x
        devia_y = float(delta_p[1]) # save delta y
        devia_z = float(delta_p[2]) # save delta z
        devia_b = float(delta_p[3]) # save delta bias
        devia_xyz=sqrt(devia_x*devia_x+devia_y*devia_y+devia_z*devia_z) # get total bias
        # print 'delta_p',delta_p
        # print 'position estimation x=',rec_ini_pos_x
        # print 'position estimation y=', rec_ini_pos_y
        # print 'position estimation Z=', rec_ini_pos_z
        # print 'position estimation b=', rec_clo_bia_b
        # print 'position estimation devia_xyz=', devia_xyz
        del gues_pseud[:] # relief gues_pseud[] list
        del pseud_error[:] # relief pseud_error[] list
    return float(rec_ini_pos_x), float(rec_ini_pos_y), float(rec_ini_pos_z), float(rec_clo_bia_b), float(devia_xyz)
'''
        GPS:     1:32
        GLONASS: 32 + 1:24
        Galileo: 57 + 1:30
        Beidou:  87 + 1:37
        QZSS:    124 + 1:4
'''


def weighted_lesat_square_GNSS_positioning_hybrid_GNSS(GNSS_one_epoch,init_x,init_y,init_z,init_b_GPS,init_b_Beidou): # least square for hybrid GNSS positioning (GPS + Beidou)
    # -------------------------------------------------initialization
    rec_ini_pos_x=float(init_x) #initial position x (m)
    rec_ini_pos_y=float(init_y) #initial position y (m)
    rec_ini_pos_z=float(init_z) #initial position z (m)
    rec_clo_bia_b_GPS=float(init_b_GPS) #initial distance bias caused by clock bias of GPS (m)
    rec_clo_bia_b_Beidou = float(init_b_Beidou)  # initial distance bias caused by clock bias of Beidou (m)
    devia_xyz            =1.0           #initial set receiver position estimation error (m)
    for coun in range(1):
    #     print '.....................................begin one iteration.................',len(GNSS_one_epoch.GNSS_Raws)
        gues_pseud=[] #guessed pseudorange
        pseud_error=[] #pseudorange error
        G_matrix_  =array([2,2,2,1,1],dtype='float') #creat G matrix to save transform parameters
        #get guessed pseudorange and pseudorange error
        for index_1 in range(len(GNSS_one_epoch.GNSS_Raws)): # index all the satellite information in one epoch
            if((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index>=88) and (GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index<=(87+37)) or
                   ((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index >= 1) and (GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index <= 32))):
                sx_1=float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_x) # satellite position x
                sy_1=float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_y) # satellite position y
                sz_1=float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_z) # satellite position z
                # print 'satellite index',GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index
                sx_1 = (sx_1 - rec_ini_pos_x) * (sx_1 - rec_ini_pos_x)  # satellite to receiver distance in x idrection
                sy_1 = (sy_1 - rec_ini_pos_y) * (sy_1 - rec_ini_pos_y)  # satellite to receiver distance in y idrection
                sz_1 = (sz_1 - rec_ini_pos_z) * (sz_1 - rec_ini_pos_z)  # satellite to receiver distance in z idrection
                sat2rec_dis=0.0 # initialize variable
                sat2rec_dis=sqrt(sx_1 + sy_1 + sz_1) # guessed pseudorange
                gues_pseud.append(sat2rec_dis) # save guessed pseudorange
                G_row = []  # G matrix row
                G_row.append(float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_x - rec_ini_pos_x) / float (sat2rec_dis) * -1) # x for G matrix row
                G_row.append(float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_y - rec_ini_pos_y) / float (sat2rec_dis) * -1) # y for G matrix row
                G_row.append(float(GNSS_one_epoch.GNSS_Raws[index_1].sat_pos_z - rec_ini_pos_z) / float (sat2rec_dis) * -1) # z for G matrix row
                if((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index>=88) and (GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index<=(87+37))):
                    element_float_GPS=0.0       # GPS element for each row
                    element_float_Beidou = 1.0  # Beidou element for each row
                    G_row.append(element_float_GPS) # save last two element for each row
                    G_row.append(element_float_Beidou)  # save last two element for each row
                    pseud_error_element = float(GNSS_one_epoch.GNSS_Raws[index_1].pseudorange) - float(sat2rec_dis) + float(rec_clo_bia_b_Beidou)  # Beidou pseudorange error
                    pseud_error.append(pseud_error_element)  # save pseudorange error
                if ((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index >= 1) and (GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index <= 32)):
                    element_float_GPS = 1.0  # GPS element for each row
                    element_float_Beidou = 0.0  # Beidou element for each row
                    G_row.append(element_float_GPS)  # save last two element for each row
                    G_row.append(element_float_Beidou)  # save last two element for each row
                    pseud_error_element = float(GNSS_one_epoch.GNSS_Raws[index_1].pseudorange) - float(sat2rec_dis) + float(rec_clo_bia_b_GPS)  # GPS pseudorange error
                    pseud_error.append(pseud_error_element)  # save pseudorange error
                # print 'length of G_row',len(G_row),'GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index',GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index
                G_matrix_=np.row_stack((G_matrix_, G_row)) # add each row to G_matrix
                del G_row[:] # relief G_row
        #get pseudorange error
        pseud_error_mat=np.array(pseud_error) #from list to array
        pseud_error_mat=pseud_error_mat.transpose() # transpose
        #get G matrix
        G_matrix_=np.delete(G_matrix_, [0], axis=0) # delete the first row of G matrix

        delta_p=np.dot((G_matrix_.transpose()),G_matrix_) # G(T) * G
        delta_p_2=np.linalg.inv(delta_p) # inverse matrix of G(T) * G
        delta_p=np.dot(delta_p_2,(G_matrix_.transpose())) # multiply (inverse matrix of G(T) * G) and G(T)
        delta_p=np.dot(delta_p,pseud_error_mat) # multiply with pseud_error_mat
        rec_ini_pos_x = rec_ini_pos_x + float(delta_p[0]) # update receiver position in x direction
        rec_ini_pos_y = rec_ini_pos_y + float(delta_p[1]) # update receiver position in y idrection
        rec_ini_pos_z = rec_ini_pos_z + float(delta_p[2]) # update receiver position in z idrection
        rec_clo_bia_b_GPS = rec_clo_bia_b_GPS + float(delta_p[3]) # update receiver clock bias of GPS in meters
        rec_clo_bia_b_Beidou = rec_clo_bia_b_Beidou + float(delta_p[4])  # update receiver clock bias of Beidou in meters
        devia_x = float(delta_p[0]) # save delta x
        devia_y = float(delta_p[1]) # save delta y
        devia_z = float(delta_p[2]) # save delta z
        devia_b_GPS = float(delta_p[3]) # save delta bias of GPS
        devia_b_Beidou = float(delta_p[4])  # save delta bias of Beidou
        devia_xyz=sqrt(devia_x*devia_x+devia_y*devia_y+devia_z*devia_z) # get total bias
        # print 'delta_p',delta_p
        # print 'position estimation x=',rec_ini_pos_x
        # print 'position estimation y=', rec_ini_pos_y
        # print 'position estimation Z=', rec_ini_pos_z
        # print 'position estimation b=', rec_clo_bia_b
        # print 'position estimation devia_xyz=', devia_xyz
        del gues_pseud[:] # relief gues_pseud[] list
        del pseud_error[:] # relief pseud_error[] list
    return float(rec_ini_pos_x), float(rec_ini_pos_y), float(rec_ini_pos_z), float(rec_clo_bia_b_GPS), float(rec_clo_bia_b_Beidou),float(devia_xyz)


def get_GPS_Beidou_satellite_numbers(GNSS_one_epoch): # get number of GPS and Beidou satellites in one epoch and save all the satellite number in list
    GPS_number=[] # create a list to save GPS satellites numbers
    Beidou_number=[] # create a list to save Beidou satellites numbers
    for index_1 in range(len(GNSS_one_epoch.GNSS_Raws)):  # index all the satellite information in one epoch
        if ((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index >= 1) and (
            GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index <= 32)): # GPS satellites index range
            GPS_number.append(float(GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index))
        if ((GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index >= 88) and (
            GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index <= (87 + 37))): # Beidou satellites index range
            Beidou_number.append(float(GNSS_one_epoch.GNSS_Raws[index_1].prn_satellites_index))
    return GPS_number,Beidou_number # return result


def callback_GNSS_data_after_NLOS_exclusion(data): # callback function: get GNSS data (after NLOS exclusion) and implement least square
    GNSS_exclusion_Array_1 = GNSS_Raw_Array()  # create an variable in GNSS_Raw_Array format to save GNSS data after exclusion
    GNSS_exclusion_Array_1 = data # get data from ros topic
    GNSS_options = 1  # 0-GPS only  1-hybrid GNSS (GPS+Beidou)
    sate_index_range = 330  # GPS only (sate_index_range=33)  Hybrid GNSS positioning (sate_index_range=330)
    GPS_satellite_number = []
    Beidou_satellite_number = []
    GPS_satellite_number, Beidou_satellite_number = get_GPS_Beidou_satellite_numbers(GNSS_exclusion_Array_1) # get satellites numbers
    iterations = 0.0  # initialize iterations times
    itera_x = 0.0  # initialize receiver's position x
    itera_y = 0.0  # initialize receiver's position y
    itera_z = 0.0  # initialize receiver's position z
    itera_b = 0.0  # initialize receiver's clock bias error of GPS only positioning in meters
    itera_bias_ = 0.0  # initialize total bias of GPS only positioning in each iteration
    itera_b_GPS = 0.0  # initialize receiver's clock bias error of GPS in Hybrid GNSS positioning in meters
    itera_b_Beidou = 0.0  # initialize receiver's clock bias error of Beidou in Hybrid GNSS positioning in meters
    if (len(GNSS_exclusion_Array_1.GNSS_Raws) > 3) and (
        GNSS_options == 0):  # at least four satellites for least square calculation with GPS only
        itera_x, itera_y, itera_z, itera_b, itera_bias_ = weighted_lesat_square_GNSS_positioning(GNSS_exclusion_Array_1, 0.1,
                                                                                                 0.1, 0.1,
                                                                                                 1.0)  # first iteration from (0.1, 0.1, 0.1, 1.0)
        while (itera_bias_ > 1e-4) and iterations < 10:  # threshold for iterations:value and times
            itera_x, itera_y, itera_z, itera_b, itera_bias_ = weighted_lesat_square_GNSS_positioning(GNSS_exclusion_Array_1,
                                                                                                     itera_x, itera_y,
                                                                                                     itera_z,
                                                                                                     itera_b)  # iteration
            iterations = iterations + 1  # add one iteration
    if (len(GNSS_exclusion_Array_1.GNSS_Raws) > 4) and (len(Beidou_satellite_number) >= 1) and (
        GNSS_options == 1):  # at least five satellites (at least one Beidou) for least square calculation with Hybrid GNSS
        itera_x, itera_y, itera_z, itera_b_GPS, itera_b_Beidou, itera_bias_ = weighted_lesat_square_GNSS_positioning_hybrid_GNSS(
            GNSS_exclusion_Array_1, 0.1, 0.1, 0.1, 1.0, 1.0)  # first iteration from (0.1, 0.1, 0.1, 1.0)
        while (itera_bias_ > 1e-4) and iterations < 10:  # threshold for iterations:value and times
            itera_x, itera_y, itera_z, itera_b_GPS, itera_b_Beidou, itera_bias_ = weighted_lesat_square_GNSS_positioning_hybrid_GNSS(
                GNSS_exclusion_Array_1, itera_x, itera_y, itera_z, itera_b_GPS, itera_b_Beidou)  # iteration
            iterations = iterations + 1  # add one iteration
    # print 'iterations after NLOS exclusion=', iterations, 'itera_bias_=', itera_bias_  # how many iterations after sucessfully least square calculation, total bioas
    iterations = 0.0  # initialize iterations variable
    # get llh information
    if (itera_x != 0):  # make sure that itera_x is not zero value, otherwise arctan(itera_y/itera_x) in xyz2llh function
        itera_xyz = []  # create a list to save ecef coordiante
        itera_xyz.append(float(itera_x))  # save x value
        itera_xyz.append(float(itera_y))  # save y value
        itera_xyz.append(float(itera_z))  # save z value
        llh_output = ecef2llh.xyz2llh(itera_xyz)  # ecef to llh coordinates
        list_lat.append(float(llh_output[0])) # save latitude
        list_lon.append(float(llh_output[1])) # save longitude
        listLatWiNLOSExc.append(float(llh_output[0])) # save latitude for csv saving
        listLonWiNLOSExc.append(float(llh_output[1]))  # save latitude for csv saving
        if (latDic.has_key(str(int(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time)))):
            truthList = []  # create a list to save ground truth
            truthList.append(float(latDic[str(int(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))]))  # get truth lat
            truthList.append(float(lonDic[str(int(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))]))  # get truth lon
            xyzTruth = llh2ecef.llh2xyz(truthList)  # get xyz from ecef
            delt_x = (xyzTruth[0] - itera_xyz[0]) * (xyzTruth[0] - itera_xyz[0])  # get delta x (Error in x)
            delt_y = (xyzTruth[1] - itera_xyz[1]) * (xyzTruth[1] - itera_xyz[1])  # get delta y (Error in y)
            delt_z = (xyzTruth[2] - itera_xyz[2]) * (xyzTruth[2] - itera_xyz[2])  # get delta z (Error in z)
            delErrTem = sqrt(delt_x + delt_y + delt_z)
            if (delErrTem > 100):
                delErrTem = 100
            plot_ExclGNSSyGtBias.append(float(delErrTem))  # save error
            plot_ExclGNSSyGtBiasTim.append(float(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))
            # print 'xyzTruth afer exclusion=', xyzTruth
        # print 'llh output after NLOS exclusion==', llh_output
    plot_excludedHdop.append(float(DopCalculation(GNSS_exclusion_Array_1)))
    plot_excludedHdopTim.append(float(GNSS_exclusion_Array_1.GNSS_Raws[0].GNSS_time))
    del GNSS_exclusion_Array_1.GNSS_Raws[:]  # relief the content


def DopCalculation(GNSS_one_epoch): # get Dop in one epoch
    H_matrix_ = array([2, 2, 2, 1], dtype='float')  # creat H matrix to save transform parameters
    Hdop_=0.0 # create an variable to save Hdop
    elemin=15.0 # minimun elevation angle
    for index_1 in range(len(GNSS_one_epoch.GNSS_Raws)):  # index all the satellite information in one epoch
        if(GNSS_one_epoch.GNSS_Raws[index_1].elevation<=elemin):
            print 'satellite elevation less than 15 degree'
            continue
        cosel=float(cos(GNSS_one_epoch.GNSS_Raws[index_1].elevation))
        sinel=float(sin(GNSS_one_epoch.GNSS_Raws[index_1].elevation))
        H_row = []  # H matrix row
        H_row.append(float(cosel * sin(GNSS_one_epoch.GNSS_Raws[index_1].azimuth)))
        H_row.append(float(cosel * cos(GNSS_one_epoch.GNSS_Raws[index_1].azimuth)))
        H_row.append(float(sinel))
        H_row.append(1.0)
        H_matrix_ = np.row_stack((H_matrix_, H_row))  # add each row to H_matrix
        del H_row[:]  # relief H_row
    # get H matrix
    H_matrix_ = np.delete(H_matrix_, [0], axis=0)  # delete the first row of H matrix
    # print 'H_matrix_',H_matrix_
    Q_matrix_ = np.dot((H_matrix_.transpose()), H_matrix_)  # H(T) * G
    Q_matrix_ = np.linalg.inv(Q_matrix_)  # inverse matrix of H(T) * G
    Hdop=float(sqrt(Q_matrix_[0,0]+Q_matrix_[1,1]))
    # print 'Q_matrix_', Q_matrix_, 'Hdop', Hdop
    return float(Hdop) # return result


if __name__ == '__main__':
    csv2topic()