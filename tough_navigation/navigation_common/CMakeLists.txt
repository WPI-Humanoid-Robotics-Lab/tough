cmake_minimum_required(VERSION 2.8.3)
project(navigation_common)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

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
  CATKIN_DEPENDS nav_msgs roscpp std_msgs sensor_msgs tf tough_common tough_controller_interface
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/frame_tracker.cpp
   src/fall_detector.cpp
   src/map_generator.cpp
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

add_executable(publish_corrected_map  src/publish_corrected_map.cpp)
target_link_libraries(publish_corrected_map  ${catkin_LIBRARIES})

add_executable(MapTransform src/MapTransform.cpp)
target_link_libraries(MapTransform  ${catkin_LIBRARIES})

add_executable(map_generator src/map_generator_node.cpp)
target_link_libraries(map_generator  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(fall_detector src/fall_detector_node.cpp)
target_link_libraries(fall_detector  ${PROJECT_NAME} ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} publish_corrected_map MapTransform map_generator fall_detector
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


