[![Build Status](https://travis-ci.org/amc-nu/HesaiLidar_Pandar64_ros.svg?branch=master)](https://travis-ci.org/amc-nu/HesaiLidar_Pandar64_ros)

# HesaiLidar_General_ROS

## About the project
HesaiLidar_General_ROS project includes the ROS Driver for：  
**PandarQT/Pandar64/Pandar40P/Pandar20A/Pandar20B/Pandar40M/PandarXT**  
LiDAR sensor manufactured by Hesai Technology.  

Developed based on [HesaiLidar_General_SDK](https://github.com/HesaiTechnology/HesaiLidar_General_SDK), After launched, the project will monitor UDP packets from Lidar,     parse data and publish point cloud frames into ROS under topic: ```/pandar```. It can also be used as an official demo showing how to work with HesaiLidar_General_SDK.

## Environment and Dependencies
**System environment requirement: Linux + ROS**  

　Recommanded:  
　Ubuntu 16.04 - with ROS kinetic desktop-full installed or  
　Ubuntu 18.04 - with ROS melodic desktop-full installed  
　Check resources on http://ros.org for installation guide 
 
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**  
```
$sudo apt install libpcap-dev libyaml-cpp-dev
```

## Download and Build

**Install `catkin_tools`**
```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```
**Download code**  
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
$ git clone https://github.com/HesaiTechnology/HesaiLidar_General_ROS.git --recursive
```
**Build**
```
$ cd ..
$ catkin config --install
$ catkin build --force-cmake
```

## Configuration 
```
 $ gedit install/share/hesai_lidar/launch/hesai_lidar.launch
```
**Reciving data from connected Lidar: config lidar ip&port, leave the pcap_file empty**
|Parameter | Default Value|
|---------|---------------|
|server_ip |192.168.1.201|
|lidar_recv_port |2368|
|gps_recv_port  |10110|
|pcap_file ||　　

Data source will be from connected Lidar when "pcap_file" set to empty
Make sure parameters above set to the same with Lidar setting

**Reciving data from pcap file: config pcap_file and correction file path**
|Parameter | Value|
|---------|---------------|
|pcap_file |pcap file path|
|lidar_correction_file |lidar correction file path|　

Data source will be from pcap file once "pcap_file" not empty 

**Reciving data from rosbag: config data_type and publish_type,leave the pcap_file empty**
|Parameter | Value|
|---------|---------------|
|pcap_file ||
|publish_type |points|　
|timestamp_type |realtime|
|data_type |rosbag|

Data source will be from rosbag when "pcap_file" is set to empty and "data_type" is set to "rosbag"
Make sure the parameter "publish_type" is set to "points"
Make sure the parameter "namespace" in file hesai_lidar.launch is same with the namespace in rosbag
Make sure the parameter "timestamp_type" is set to "realtime" for the type of reference time published to ROS  

## Run

1. Make sure current path in the `rosworkspace` directory
```
$ source install/setup.bash
```
```
for PandarQT
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarQT" frame_id:="PandarQT"
for Pandar64
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar64" frame_id:="Pandar64"
for Pandar20A
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar20A" frame_id:="Pandar20A"
for Pandar20B
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar20B" frame_id:="Pandar20B"
for Pandar40P
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar40P" frame_id:="Pandar40P"
for Pandar40M
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar40M" frame_id:="Pandar40M"
for PandarXT-32
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT-32" frame_id:="PandarXT-32"
for PandarXT-16
$ roslaunch hesai_lidar hesai_lidar.launch lidar_type:="PandarXT-16" frame_id:="PandarXT-16"
```
2. The driver will publish PointCloud messages to the topic `/pandar`  
3. Open Rviz and add display by topic  
4. Change fixed frame to frame_id to view published point clouds  
