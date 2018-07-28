#! /usr/bin/env python
# -*- coding=utf-8 -*-
# @Author WEN WS Ph.D
# get ground truth from csv file provided by Dr.Li-Ta HSU
import csv # csv reading needed library
import pandas as pd
filename="groundTruth.csv"
csv_reader=csv.reader(open('COM9_171201_073906_gt.csv', 'r')) #read csv context to csv_reader variable
time=[]
lat=[]
lon=[]
static_lat=22.303953
static_lon=114.181925
static_hight=16.0
startTim=129780
endTim  =130044
# for dynamic experiment
# for csv_row in csv_reader:
#     time.append(float(csv_row[0]))
#     lat.append(csv_row[3])
#     lon.append(csv_row[4])
# print 'len(lat)',len(lat),type(lat)
# print 'len(lon)',len(lon),type(lon)

# for static experiment
for cou in range(endTim-startTim+1):
    time.append(float(startTim+cou))
    lat.append(static_lat)
    lon.append(static_lon)
print 'len(lat)',len(lat),type(lat)
print 'len(lon)',len(lon),type(lon)


if len(lon) != len(lat):
    print 'lon != lat nums'
with open(filename, 'w') as file_object: # save listLatRaw and listLonRaw to csv file
    for sav_idx in range(len(time)):
        str1=str(int(time[sav_idx]))+','+str(lat[sav_idx])+','+str(lon[sav_idx])
        file_object.write(str1)
        file_object.write('\n')

csv_readerGt=csv.reader(open('/home/wenws/23_pointcloud2laserscan/src/pointcloud_to_laserscan/src/groundTruth.csv', 'r')) #read csv context to csv_reader variable
latDic={}
lonDic={}
for csv_row in csv_readerGt:
    latDic[str(csv_row[0])] =  str(csv_row[1])
    lonDic[str(csv_row[0])] =  str(csv_row[2])
print latDic[str(startTim)]