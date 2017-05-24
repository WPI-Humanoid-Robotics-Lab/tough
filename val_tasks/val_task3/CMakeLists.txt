cmake_minimum_required(VERSION 2.8.3)
project(val_task3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  decision_making
  decision_making_parser
  dynamic_reconfigure
  roscpp
  val_footstep
  val_control
  perception_common
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  val_task_common
  val_moveit_planners

)

## System dependencies are found with CMake's conventions

find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES val_task3
 CATKIN_DEPENDS roscpp decision_making decision_making_parser dynamic_reconfigure val_footstep cv_bridge image_transport pcl_conversions pcl_ros sensor_msgs std_msgs val_control
 DEPENDS system_lib
 #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
         src/val_task3SM.cpp
         src/val_task3.cpp
         src/val_stairclimb.cpp
         src/stair_detector.cpp
         src/steps_detector.cpp
         src/leak_detector_grabber.cpp
         src/val_task3_utils.cpp
)

add_executable(stair_climb_node src/val_stairclimb.cpp)
target_link_libraries(stair_climb_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


add_executable(stair_detector_node        src/stair_detector_node.cpp)
target_link_libraries(stair_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME})

add_executable(steps_detector_node        src/steps_detector_node.cpp)
target_link_libraries(steps_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES} ${PCL_LIBRARIES_DIRS}   ${PROJECT_NAME})

add_executable(leak_detector_grabber_node src/leak_detector_grabber_node.cpp)
target_link_libraries(leak_detector_grabber_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(val_task3_node            src/val_task3_node.cpp)

add_dependencies(val_task3_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg)

target_link_libraries(val_task3_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

decision_making_parsing(src/val_task3SM.cpp)

## Mark executables and/or libraries for installation
 install(TARGETS  ${PROJECT_NAME}_node
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
