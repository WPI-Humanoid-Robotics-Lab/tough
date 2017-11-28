cmake_minimum_required(VERSION 2.8.3)
project(src_task3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")

add_definitions(-D$ENV{ROBOT_NAME})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  decision_making
  decision_making_parser
  dynamic_reconfigure
  roscpp
  tough_footstep
  tough_controller_interface
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
  tough_moveit_planners
  tough_control_common
)

## System dependencies are found with CMake's conventions

find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/task3_parameters.cfg
 )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES src_task3
 CATKIN_DEPENDS roscpp decision_making decision_making_parser dynamic_reconfigure tough_footstep cv_bridge image_transport pcl_conversions pcl_ros sensor_msgs std_msgs tough_controller_interface
 DEPENDS system_lib
 #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  include
  cfg/cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
         src/val_task3SM.cpp
         src/val_task3.cpp
         src/val_stairclimb.cpp
         src/stair_detector.cpp
         src/stair_detector_2.cpp
         src/steps_detector.cpp
         src/table_detector.cpp
         src/leak_detector_grabber.cpp
         src/door_valve_detector.cpp
         src/val_task3_utils.cpp
         src/rotate_valve.cpp
         src/door_opener.cpp
         src/climb_stairs.cpp
         src/leak_detector.cpp
)

add_executable(stair_climb_node src/val_stairclimb.cpp)
target_link_libraries(stair_climb_node ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(door_valve_detector_node       src/door_valve_detector_node.cpp)
target_link_libraries(door_valve_detector_node   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARY_DIRS} ${PROJECT_NAME})

add_executable(stair_detector_node        src/stair_detector_node.cpp)
target_link_libraries(stair_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME})

add_executable(stair_detector_2_node        src/stair_detector_2_node.cpp)
target_link_libraries(stair_detector_2_node ${catkin_LIBRARIES}   ${PCL_LIBRARIES}   ${PROJECT_NAME})

add_executable(table_detector_node        src/table_detector_node.cpp)
target_link_libraries(table_detector_node ${catkin_LIBRARIES}   ${PCL_LIBRARIES}   ${PROJECT_NAME})

add_executable(steps_detector_node        src/steps_detector_node.cpp)
target_link_libraries(steps_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES} ${PCL_LIBRARIES_DIRS}   ${PROJECT_NAME})

add_executable(leak_detector_grabber_node src/leak_detector_grabber_node.cpp)
target_link_libraries(leak_detector_grabber_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(grab_valve_node src/grab_valve_node.cpp)
target_link_libraries(grab_valve_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(door_opener_node src/door_opener_node.cpp)
target_link_libraries(door_opener_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(door_task_node src/door_task_node.cpp)
target_link_libraries(door_task_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(blind_goal_finder src/blind_goal_finder.cpp)
target_link_libraries(blind_goal_finder ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(climb_stairs_node src/climb_stairs_node.cpp)
target_link_libraries(climb_stairs_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(leak_detector_node src/leak_detector_node.cpp)
target_link_libraries(leak_detector_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node         src/val_task3_node.cpp)

add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg
 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
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
