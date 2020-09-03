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


## Config
```
 $ cd rosworkspace/install/share/hesai_lidar/launch
```
open hesai_lidar.launch to set configuration parameters
### Reciving data from connected Lidar: config lidar ip&port, leave the pcap_file empty
|Parameter | Default Value|
|---------|---------------|
|server_ip |192.168.1.201|
|lidar_recv_port |2368|
|gps_recv_port  |10110|
|pcap_file ||

Data source will be from connected Lidar when "pcap_file" set to empty

### Reciving data from pcap file: config pcap_file and correction file path
|Parameter | Value|
|---------|---------------|
|pcap_file |pcap file path|
|lidar_correction_file |lidar correction file path|

Data source will be from pcap file once "pcap_file" not empty 


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
4. Change fixed frame to lidar_type to view published point clouds.
