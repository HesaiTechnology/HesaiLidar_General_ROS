# HesaiLidar_General_SDK
Hesai Lidar General SDK for Pandar40P/Pandar64/Pandar20A/Pandar20B/PandarQT/Pandar40M/PandarXT
## Clone
```
git clone https://github.com/HesaiTechnology/HesaiLidar_General_SDK.git
```
## Build
```
cd <project>
mkdir build
cd build
cmake ..
make
```
## Add to your project
### Cmake
```
add_subdirectory(<path_to>HesaiLidar_General_SDK)

include_directories(
	<path_to>HesaiLidar_General_SDK/include
	<path_to>HesaiLidar_General_SDK/src/PandarGeneralRaw/include
)

target_link_libraries(<Your project>
  PandarGeneralSDK
)
```
### C++
```
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
// for Pandar40P
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40P"));
// for Pandar64
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar64"));
// for Pandar20A
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar20A"));
// for Pandar20B
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar20B"));
// for PandarQT
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarQT"));
// for Pandar40M
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40M"));
// for PandarXT-32
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarXT-32"));
// for PandarXT-16
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarXT-16"));
```
