# Package for: Analysis of static objects on GNSS single point positioning

- Point clouds based object detection
- Fault detection and exclusion (FDE)
- GNSS single point positioning with weighted least square (WLS) in python
- Skyplot visualization
- Positioning result visualization in python

## Spec Recommendation

- Number of CPU cores: 4
- RAM size: 16GB
- Storage size: 30GB in SSD

## Requirements

- ROS jade (Ubuntu 14.04)
- Qt 5.2.1 or higher

### Install dependencies for Ubuntu 14.04 jade

install all the dependency when needed



## How to Build

```
$ cd $HOME
$ mkdir 3_nlosMovobj/src
$ cd 3_nlosMovobj/src
$ git clone https://github.com/weisongwen/IONPlans2018.git
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

## How to Start

```
$ cd $HOME/3_nlosMovobj/src
$ ./all.sh
```

## How to use this for your data

The data is saved in Dropbox. The data for public will be opened soon,


## Research Papers for Reference

1. Wen, Weisong, Guohao Zhang, and Li-Ta Hsu. "Exclusion of GNSS NLOS receptions caused by dynamic objects in heavy traffic urban scenarios using real-time 3D point cloud: An approach without 3D maps." Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION. IEEE, 2018. (https://ieeexplore.ieee.org/abstract/document/8373377/)

