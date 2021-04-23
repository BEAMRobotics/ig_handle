cmake_minimum_required(VERSION 2.8.3)
project(ig_handle)

## c++11 feature
SET(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
add_definitions(-DBOOST_LOG_DYN_LINK)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  wfov_camera_msgs
)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    sensor_msgs
    wfov_camera_msgs
)

## Build ##
## Main library
add_library(restamp_nodelets
  src/generate_nodelets.cpp
  )
target_include_directories(restamp_nodelets
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(restamp_nodelets
  ${catkin_LIBRARIES}
  )

## Node version of restamper
add_executable(restamp_node
  src/restamp_node.cpp
)
target_include_directories(restamp_node
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(restamp_node
  ${catkin_LIBRARIES}
)

## Install ##
install(TARGETS 
  restamp_node
  restamp_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
