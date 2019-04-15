[![Build Status](https://travis-ci.org/amc-nu/HesaiLidar_Pandar64_ros.svg?branch=master)](https://travis-ci.org/amc-nu/HesaiLidar_Pandar64_ros)

# Hesai Pandar40P 

This repository includes the ROS Driver for the Pandar64 LiDAR sensor manufactured by Hesai Technology.


## How to Build

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

## How to Launch

1. While in the `rosworkspace` directory.
```
$ source install/setup.bash
$ roslaunch hesai_lidar hesai_lidar64.launch
```
2. The driver will publish a PointCloud message in the topic.
```
/points_raw
```
3. Open Rviz and add display by topic.
4. Change fixed frame to `pandar`.

LiDAR default IP address is 192.168.1.201

## Some of the available parameters in the Launch file

|Parameter | Default Value|
|---------|---------------|
|lidar_recv_port |2368|
|gps_recv_port  |10110|
|start_angle |0|

