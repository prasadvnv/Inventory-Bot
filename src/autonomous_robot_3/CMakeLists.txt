cmake_minimum_required(VERSION 2.8.3)
project(autonomous_robot_3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES autonomous_robot_3
  CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

