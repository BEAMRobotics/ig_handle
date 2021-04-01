cmake_minimum_required(VERSION 2.8.3)
project(ig_handle)

## c++11 feature
SET(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
add_definitions(-DBOOST_LOG_DYN_LINK)
set(catkin_deps roscpp )

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

## Build ##
 
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)
