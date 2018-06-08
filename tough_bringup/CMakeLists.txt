cmake_minimum_required(VERSION 2.8.3)
project(tough_bringup)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  tough_common
  tough_control_common
  tough_controller_interface
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS tough_common tough_control_common tough_controller_interface
  DEPENDS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
