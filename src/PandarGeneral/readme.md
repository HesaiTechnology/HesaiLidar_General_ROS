# Hesai Pandar General SDK

This repository includes the SDK for the PandarQT/Pandar64/Pandar20A/Pandar20B/Pandar40P/Pandar40M LiDAR sensor manufactured by Hesai Technology.

## How to Build

```
$ sudo apt-get update
$ sudo apt-get install libpcl-dev libboost-all-dev
```

### Compile

```
$ mkdir -p build
$ cd build
$ cmake ..
$ make
```

## Add to your project

```
add_subdirectory(<path to>PandarGeneralSDK)
target_link_libraries(${YOUR_PROJECT_NAME}
    ...
    PandarGeneralSDK
    ...
  )
```

