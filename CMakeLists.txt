cmake_minimum_required(VERSION 2.8.3)
project(ig_handle)

## c++11 feature
SET(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    std_msgs
    sensor_msgs
)
