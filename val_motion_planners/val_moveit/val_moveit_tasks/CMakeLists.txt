cmake_minimum_required(VERSION 2.8.3)
project(val_moveit_tasks)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter")
endif()

add_definitions(-D$ENV{ROBOT_NAME})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  val_moveit_config
  moveit_commander
  moveit_core
  moveit_fake_controller_manager
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_benchmarks
  #moveit_ros_benchmarks_gui
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
#  descartes_core
#  descartes_planner
#  descartes_moveit
#  descartes_trajectory
#  descartes_utilities
  geometry_msgs

  val_gui
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES val_moveit_tasks
  CATKIN_DEPENDS moveit_commander moveit_config_trac_ik moveit_core moveit_fake_controller_manager moveit_msgs moveit_planners_ompl moveit_ros_benchmarks moveit_ros_benchmarks_gui moveit_ros_control_interface moveit_ros_manipulation moveit_ros_move_group moveit_ros_perception moveit_ros_planning moveit_ros_planning_interface moveit_ros_robot_interaction moveit_ros_visualization moveit_ros_warehouse moveit_simple_controller_manager trac_ik_examples trac_ik_kinematics_plugin trac_ik_lib
  DEPENDS system_lib
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES val_moveit_tasks
#  CATKIN_DEPENDS moveit_commander moveit_config_trac_ik moveit_core moveit_fake_controller_manager moveit_msgs moveit_planners_ompl moveit_ros_benchmarks moveit_ros_benchmarks_gui moveit_ros_control_interface moveit_ros_manipulation moveit_ros_move_group moveit_ros_perception moveit_ros_planning moveit_ros_planning_interface moveit_ros_robot_interaction moveit_ros_visualization moveit_ros_warehouse moveit_simple_controller_manager trac_ik_examples trac_ik_kinematics_plugin trac_ik_lib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


## Declare a C++ executable
add_executable(raise_arm_move_group src/raise_arm_move_group.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(raise_arm_move_group ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(raise_arm_move_group
   ${catkin_LIBRARIES}
 )

add_executable(test_moveit src/test_moveit.cpp)
target_link_libraries(test_moveit
   ${catkin_LIBRARIES}
 )
 add_executable(val_pose src/val_pose.cpp)
 target_link_libraries(val_pose
    ${catkin_LIBRARIES}
  )

  add_executable(test_circle src/test_circle.cpp)
  target_link_libraries(test_circle
     ${catkin_LIBRARIES}
     val_marker
   )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS val_moveit_tasks val_moveit_tasks_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_moveit_tasks.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
