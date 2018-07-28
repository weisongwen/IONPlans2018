#!/usr/bin/env bash 
# gnome-terminal -x bash -c "./pointcloud_to_laserscan_processing.sh;exec bash;"
# gnome-terminal -x bash -c "./skyplot_visualization.sh;exec bash;"
gnome-terminal -x bash -c "./openbag.sh;exec bash;"
gnome-terminal -x bash -c "./double_decker_clustering.sh;exec bash;"
gnome-terminal -x bash -c "./puNlosExclusionS.sh;exec bash;"

