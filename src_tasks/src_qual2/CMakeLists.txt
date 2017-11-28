cmake_minimum_required(VERSION 2.8.3)
project(src_qual2)

add_definitions(-std=c++11)

#set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tough_common
  tough_controller_interface
  tough_footstep
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES src_qual2
  CATKIN_DEPENDS roscpp std_msgs tf tough_common tough_controller_interface tough_footstep
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(src_qual2_node src/src_qual2.cpp)
add_dependencies(src_qual2_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(src_qual2_node ${catkin_LIBRARIES})

