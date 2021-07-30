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
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES arduino_sync.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs sensor_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    std_msgs
    sensor_msgs
    actionlib_msgs
    message_runtime
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

add_executable(arduino_sync_server
  src/arduino_sync_server.cpp
)
target_include_directories(arduino_sync_server
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${catkin_EXPORTED_TARGETS} 

)
target_link_libraries(arduino_sync_server
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(arduino_sync_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} 
)

add_executable(timestring src/timestring.cpp)
target_include_directories(timestring
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(timestring ${catkin_LIBRARIES})


## Install ##
install(TARGETS 
  restamp_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
