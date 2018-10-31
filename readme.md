## Dependency
```
1. ROS
2. sudo apt install libpcap-dev libyaml-cpp-dev
```

## Build
```
mkdir -p rosworkspace/src ; cd rosworkspace/src
git clone https://github.com/HesaiTechnology/HesaiLidar_Pandar64_ros.git --recursive
cd ../
catkin_make
source ./devel/setup.sh
```

## Run
### Pandar64
```
roslaunch hesai_lidar hesai_lidar64.launch
```

There is one node of Hesai Lidar ROS
```
/pandar_points
```

## Parameters:
```
	<arg name="server_ip" default="192.168.1.201"/> lidar's ip
	<arg name="lidar_recv_port"  default="2368"/>   lidar's port
	<arg name="gps_port"  default="10110"/>         gps's port
	<arg name="start_angle"  default="0"/>          lidar's start angle

  ......

```
