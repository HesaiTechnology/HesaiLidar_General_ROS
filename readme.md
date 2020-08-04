[![Build Status](https://travis-ci.org/amc-nu/HesaiLidar_Pandar64_ros.svg?branch=master)](https://travis-ci.org/amc-nu/HesaiLidar_Pandar64_ros)

# HesaiLidar_General_ROS

This repository includes the ROS Driver for PandarQT/Pandar64/Pandar40P/Pandar20A/Pandar20B/Pandar40M/PandarXT LiDAR sensor manufactured by Hesai Technology.


## Build

### Install `catkin_tools`

```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

### Compile

1. Create ROS Workspace. i.e. `rosworkspace`
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
```

2. Clone recursively this repository.
3. Install required dependencies with the help of `rosdep` 
```
$ cd ..
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 
```
4. Compile
```
$ catkin config --install
$ catkin build --force-cmake
```

## Run

1. While in the `rosworkspace` directory.
```
$ source install/setup.bash
for PandarQT
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarQT"
for Pandar64
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar64"
for Pandar20A
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar20A"
for Pandar20B
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar20B"
for Pandar40P
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar40P"
for Pandar40M
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar40M"
for PandarXT
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT"
```
2. The driver will publish a PointCloud message in the topic.
```
/pandar
```
3. Open Rviz and add display by topic.
4. Change fixed frame to lidar_type.

LiDAR default IP address is 192.168.1.201

## Some of the available parameters in the Launch file

|Parameter | Default Value|
|---------|---------------|
|lidar_recv_port |2368|
|gps_recv_port  |10110|
|start_angle |0|

