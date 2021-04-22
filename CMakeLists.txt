cmake_minimum_required(VERSION 2.8.3)
project(ig_handle)

## c++11 feature
SET(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
add_definitions(-DBOOST_LOG_DYN_LINK)
set(catkin_deps roscpp )

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

## Build ##
 
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)


add_library(restamp_nodelet src/restamp_nodelet.cpp)
target_link_libraries(restamp_nodelet ${catkin_LIBRARIES})

add_executable(restamp_node src/restamp_node.cpp)
target_link_libraries(restamp_node ${catkin_LIBRARIES})

## Install ##
install(TARGETS 
  restamp_node
  restamp_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
