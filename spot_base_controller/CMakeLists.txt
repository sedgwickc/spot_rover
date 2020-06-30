cmake_minimum_required(VERSION 2.8.3)
project(spot_base_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs
)
# Build MotorControl library
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${spot_base_controller_INCLUDE_DIRS}
#)
#add_library(MotorControl src/MotorControl.cpp)
#target_link_libraries(MotorControl roboticscape ${catkin_LIBRARIES})

# build ROS spot_base_controller node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${spot_base_controller_INCLUDE_DIRS}
)
add_executable(spot_base_controller src/spot_base_controller.cpp src/MotorControl.cpp)
target_link_libraries(spot_base_controller roboticscape ${catkin_LIBRARIES})

