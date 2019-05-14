cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(pcl_visualizer_viewports)


find_package(PCL 1.6 REQUIRED)
find_package(OPENCV 4.1.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo main.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})