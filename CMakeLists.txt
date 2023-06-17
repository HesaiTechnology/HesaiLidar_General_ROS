cmake_minimum_required(VERSION 3.5)
project(hesai_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package( Boost REQUIRED  COMPONENTS thread)
find_package(Boost REQUIRED COMPONENTS system)
# find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_c REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
# install(FILES DESTINATION share/${PROJECT_NAME})
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(rosidl_default_runtime)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
# find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
# find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PandarPacket.msg"
  "msg/PandarScan.msg"
  DEPENDENCIES builtin_interfaces 
)
find_package(PCL REQUIRED COMPONENTS io)
ament_export_dependencies(rosidl_default_runtime)
##PandarGeneralRaw
add_library(PandarGeneral
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/input.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral_internal.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pcap_reader.cpp
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral.cc
)
target_include_directories(PandarGeneral PRIVATE
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    # ${ament_INCLUDE_DIRS}
)
target_link_libraries(PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    Boost::thread
    pcap
    # ${ament_LIBRARIES}
    ${tf2_ros_LIBRARIES}
)

rosidl_target_interfaces(PandarGeneral  ${PROJECT_NAME} "rosidl_typesupport_cpp")

###PandarGeneralSDK
add_library(PandarGeneralSDK SHARED
    src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc
    src/HesaiLidar_General_SDK/src/tcp_command_client.c
    src/HesaiLidar_General_SDK/src/util.c
)
target_include_directories(PandarGeneralSDK PRIVATE
    src/HesaiLidar_General_SDK/
    src/HesaiLidar_General_SDK/include/
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(PandarGeneralSDK 
    PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    # ${YAML_CPP_LIBRARIES}
    Boost::thread
    pcap
)
rosidl_target_interfaces(PandarGeneralSDK   ${PROJECT_NAME} "rosidl_typesupport_cpp")


##hesai node
add_executable(hesai_lidar_node
    src/main_ros2.cc
)
ament_target_dependencies(hesai_lidar_node
  "geometry_msgs"  # maybe don't need this
  "ament_index_cpp"
  "pcl_conversions"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "image_transport"
  "rclcpp_components"
)

target_include_directories(hesai_lidar_node PRIVATE
    src/HesaiLidar_General_SDK/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(hesai_lidar_node 
    ${PCL_IO_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    PandarGeneralSDK
)

rosidl_target_interfaces(hesai_lidar_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS hesai_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
