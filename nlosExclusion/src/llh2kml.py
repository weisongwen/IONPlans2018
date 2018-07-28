#! /usr/bin/env python
# -*- coding=utf-8 -*-
# @Author virqin.github.io
from lxml import etree  #将KML节点输出为字符串
import xlrd             #操作Excel
from pykml.factory import KML_ElementMaker as KML #使用factory模块
import csv # csv reading needed library
import pandas as pd
csv_reader=csv.reader(open('llhWitNLOSExlusion.csv', 'r')) #read csv context to csv_reader variable
lat=[]
lon=[]
#取得第一和第二列全部的值
for csv_row in csv_reader:
    lat.append(csv_row[0])
    lon.append(csv_row[1])
print 'len(lat)',len(lat),type(lat)
print 'len(lon)',len(lon),type(lon)
#简单判断文件中的经纬度个数是否一致
if len(lon) != len(lat):
    print 'lon != lat nums'
#使用第一个点创建Folder
fold = KML.Folder(KML.Placemark(
    KML.Point(KML.coordinates(str(lon[0]) +','+ str(lat[0]) +',0'))
    )
)
#将剩余的点追加到Folder中
for i in range(1,len(lon)):
    fold.append(KML.Placemark(
    KML.Point(KML.coordinates(str(lon[i]) +','+ str(lat[i]) +',0')))
    )
#使用etree将KML节点输出为字符串数据
content = etree.tostring(etree.ElementTree(fold),pretty_print=True)
#保存到文件，然后就可以在Google地球中打开了
with open('llhWitNLOSExclusion.kml', 'w') as fp:
    fp.write(content)


csv_reader=csv.reader(open('llhRaw.csv', 'r')) #read csv context to csv_reader variable
lat=[]
lon=[]
#取得第一和第二列全部的值
for csv_row in csv_reader:
    lat.append(csv_row[0])
    lon.append(csv_row[1])
print 'len(lat)',len(lat),lat
print 'len(lon)',len(lon),lon
#简单判断文件中的经纬度个数是否一致
if len(lon) != len(lat):
    print 'lon != lat nums'
#使用第一个点创建Folder
fold = KML.Folder(KML.Placemark(
    KML.Point(KML.coordinates(str(lon[0]) +','+ str(lat[0]) +',0'))
    )
)

#将剩余的点追加到Folder中
for i in range(1,len(lon)):
    fold.append(KML.Placemark(
    KML.Point(KML.coordinates(str(lon[i]) +','+ str(lat[i]) +',0')))
    )

#使用etree将KML节点输出为字符串数据
content = etree.tostring(etree.ElementTree(fold),pretty_print=True)

#保存到文件，然后就可以在Google地球中打开了
with open('llhRAW.kml', 'w') as fp:
    fp.write(content)