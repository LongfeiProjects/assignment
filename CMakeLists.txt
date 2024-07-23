cmake_minimum_required(VERSION 2.8.3)
project(assignment)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  dsr_msgs  
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/poc_pick_cube.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
