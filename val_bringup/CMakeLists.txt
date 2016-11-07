cmake_minimum_required(VERSION 2.8.3)
project(val_bringup)

find_package(srcsim REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS 
  DEPENDS srcsim
)

