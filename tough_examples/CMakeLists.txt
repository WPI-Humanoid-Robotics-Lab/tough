cmake_minimum_required(VERSION 2.8.3)
project(tough_examples)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tough_controller_interface
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tough_examples
  CATKIN_DEPENDS roscpp rospy std_msgs tough_controller_interface
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Reset Robot
add_executable( reset_robot src/reset_robot.cpp)
add_dependencies(reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reset_robot ${catkin_LIBRARIES})


## Test ARM
add_executable( test_arm src/test_arm.cpp)
add_dependencies(test_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm ${catkin_LIBRARIES})


## Test head navigation
add_executable( test_head_navigation src/test_head_navigation.cpp)
add_dependencies(test_head_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_head_navigation ${catkin_LIBRARIES})

