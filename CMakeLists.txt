cmake_minimum_required(VERSION 3.0.2)
project(ig_handle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ig_handle
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_virtual_machine src/sonar_virtual_machine.cpp)

target_link_libraries(sonar_virtual_machine
   ${catkin_LIBRARIES}
)