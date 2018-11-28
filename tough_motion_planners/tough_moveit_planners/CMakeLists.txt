cmake_minimum_required(VERSION 2.8.3)
project(tough_moveit_planners)

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  moveit_core
  moveit_fake_controller_manager
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_benchmarks
#  moveit_ros_benchmarks_gui
  moveit_ros_control_interface
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_visualization
  moveit_ros_warehouse
  moveit_simple_controller_manager
  trac_ik_examples
  trac_ik_kinematics_plugin
  trac_ik_lib
  tough_controller_interface
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_moveit_planners
  CATKIN_DEPENDS moveit_commander moveit_core moveit_fake_controller_manager moveit_msgs moveit_planners_ompl moveit_ros_control_interface moveit_ros_manipulation moveit_ros_move_group moveit_ros_perception moveit_ros_planning moveit_ros_planning_interface moveit_ros_robot_interaction moveit_ros_visualization moveit_ros_warehouse moveit_simple_controller_manager trac_ik_examples trac_ik_kinematics_plugin trac_ik_lib tough_controller_interface
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} include )

add_library(${PROJECT_NAME}
   src/tough_cartesian_planner.cpp
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}
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


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_moveit_planners.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
