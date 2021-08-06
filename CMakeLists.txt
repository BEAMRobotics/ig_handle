cmake_minimum_required(VERSION 2.8.3)
project(ig_handle)

## c++11 feature
SET(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
add_definitions(-DBOOST_LOG_DYN_LINK)

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

## Build ##
## Main library

add_executable(timestring src/timestring.cpp)
target_include_directories(timestring
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(timestring ${catkin_LIBRARIES})

