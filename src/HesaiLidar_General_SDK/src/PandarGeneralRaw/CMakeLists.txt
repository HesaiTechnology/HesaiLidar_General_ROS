cmake_minimum_required(VERSION 2.8)
project(PandarGeneral)


find_package( Boost REQUIRED )
find_package(PCL REQUIRED COMPONENTS common)

set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")


include_directories(
	.
	include
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_library( ${PROJECT_NAME}
	src/input.cc
	src/pandarGeneral_internal.cc
	src/pandarGeneral.cc
    src/pcap_reader.cpp
)

target_link_libraries(${PROJECT_NAME}
		${Boost_LIBRARIES}
		${PCL_IO_LIBRARIES}
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

