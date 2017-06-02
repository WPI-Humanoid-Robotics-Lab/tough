cmake_minimum_required(VERSION 2.8.3)
project(val_task2)

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
  val_controllers
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
  val_control_common
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


generate_dynamic_reconfigure_options(
  cfg/task2_parameters.cfg
 )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES val_task2
 CATKIN_DEPENDS roscpp decision_making decision_making_parser dynamic_reconfigure val_footstep cv_bridge image_transport pcl_conversions pcl_ros sensor_msgs std_msgs val_controllers
 DEPENDS system_lib
 #DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
 cfg/cpp
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
            src/val_task2.cpp
            src/val_task2SM.cpp
	    src/button_detector.cpp
            src/button_press.cpp
            src/solar_panel_grasp.cpp
            src/val_rover_detection.cpp
	    src/cable_detector.cpp
            src/plug_detector.cpp
            src/val_solar_detection.cpp
            src/val_solar_panel_detector.cpp
            src/val_task2_utils.cpp
            src/cable_task.cpp
            src/array_table_detector.cpp
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

add_executable(solar_panel_detector_node       src/val_solar_panel_detector_node.cpp)
target_link_libraries(solar_panel_detector_node   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARY_DIRS} ${PROJECT_NAME})


add_executable(rover_detection_node       src/val_rover_detection_node.cpp)
target_link_libraries(rover_detection_node   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARY_DIRS} ${PROJECT_NAME})


add_executable(solar_detection_node       src/val_solar_detection_node.cpp)
target_link_libraries(solar_detection_node   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARY_DIRS} ${PROJECT_NAME})

add_executable(button_detector_node  src/button_detector_node.cpp)
target_link_libraries(button_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME} ${PCL_LIBRARY_DIRS})

add_executable(button_press_node        src/button_press_node.cpp)
target_link_libraries(button_press_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME})

add_executable(cable_detector_node        src/cable_detector_node.cpp)
target_link_libraries(cable_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME}
)

add_executable(array_table_detector_node        src/array_table_detector_node.cpp)
target_link_libraries(array_table_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME}
)

add_executable(plug_detector_node	src/plug_detector_node.cpp)

target_link_libraries(plug_detector_node ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES}   ${PROJECT_NAME})

add_executable(val_task2_node            src/val_task2_node.cpp)

add_executable(solar_panel_grasp_node src/solar_panel_grasp_node.cpp)
target_link_libraries(solar_panel_grasp_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(cable_grabber_node src/cable_grabber_node.cpp)
target_link_libraries(cable_grabber_node  ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_dependencies(cable_grabber_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg)

add_executable(insert_cable_node src/insert_cable_node.cpp)
target_link_libraries(insert_cable_node  ${catkin_LIBRARIES} ${PROJECT_NAME} )

add_dependencies(insert_cable_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg)

add_dependencies(val_task2_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg)

## Specify libraries to link a library or executable target against
 target_link_libraries(val_task2_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

decision_making_parsing(src/val_task2SM.cpp)

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
