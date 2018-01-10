cmake_minimum_required(VERSION 2.8.3)
project(navigation_common)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  tough_common
  tough_controller_interface
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation_common
  CATKIN_DEPENDS nav_msgs roscpp std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
 add_library(navigation_common
   src/frame_tracking.cpp
   src/fall_detector.cpp
   src/map_generator.cpp
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

## Declare a C++ executable
# add_executable(navigation_common_node src/navigation_common_node.cpp)

add_executable(publish_corrected_map  src/publish_corrected_map.cpp)
target_link_libraries(publish_corrected_map  ${catkin_LIBRARIES})

add_executable(MapTransform src/MapTransform.cpp)
target_link_libraries(MapTransform  ${catkin_LIBRARIES})

add_executable(map_generator src/map_generator_node.cpp)
target_link_libraries(map_generator  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(fall_detector src/fall_detector_node.cpp)
target_link_libraries(fall_detector  ${PROJECT_NAME} ${catkin_LIBRARIES})

