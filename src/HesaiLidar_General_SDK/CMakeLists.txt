cmake_minimum_required(VERSION 2.8)
project(PandarGeneralSDK)

find_package( Boost REQUIRED  COMPONENTS thread)
find_package( PCL REQUIRED COMPONENTS common )

# set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

add_subdirectory(src/PandarGeneralRaw)

include_directories(
    .
    include
    src/PandarGeneralRaw/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} SHARED
    src/pandarGeneral_sdk.cc
    src/tcp_command_client.c
    src/util.c
)

target_link_libraries(${PROJECT_NAME} 
    PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    Boost::thread
    pcap
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(PCL REQUIRED)
    add_executable(PandarGeneralTest
        test/test.cc
    )

    target_link_libraries(PandarGeneralTest
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        Boost::thread
    )
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

