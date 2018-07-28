#!/usr/bin/env bash

cd /home/wenws/23_pointcloud2laserscan
source /home/wenws/23_pointcloud2laserscan/devel/setup.bash
rosrun pointcloud_to_laserscan skyplot_python_subscribe.py
#localization_map.bt
#autoware-170601.bt