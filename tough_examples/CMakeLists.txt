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

######################
## Control examples ##
######################


## Reset Robot
add_executable( reset_robot src/control_examples/reset_robot.cpp)
add_dependencies(reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reset_robot ${catkin_LIBRARIES})


## Test ARM
add_executable( arm_example src/control_examples/arm_example.cpp)
add_dependencies(arm_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_example ${catkin_LIBRARIES})


## Test head navigation
add_executable( head_navigation_example src/control_examples/head_navigation_example.cpp)
add_dependencies(head_navigation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_navigation_example ${catkin_LIBRARIES})

## Test arm navigation
add_executable(arm_navigation_example src/control_examples/arm_navigation_example.cpp)
add_dependencies(arm_navigation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_navigation_example  ${catkin_LIBRARIES})

## Test chest
add_executable(chest_navigation_example src/control_examples/chest_navigation_example.cpp)
add_dependencies(chest_navigation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chest_navigation_example  ${catkin_LIBRARIES})


## Test nudgeLocal
add_executable(nudge_local_example src/control_examples/nudge_local_example.cpp)
add_dependencies(nudge_local_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nudge_local_example  ${catkin_LIBRARIES})


## Test pelvis
add_executable(pelvis_navigation_example src/control_examples/pelvis_navigation_example.cpp)
add_dependencies(pelvis_navigation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pelvis_navigation_example  ${catkin_LIBRARIES})


## Test neck
add_executable(neck_navigation_example src/control_examples/neck_navigation_example.cpp)
add_dependencies(neck_navigation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(neck_navigation_example  ${catkin_LIBRARIES})

## Test wrist
add_executable(wrist_example src/control_examples/wrist_example.cpp)
add_dependencies(wrist_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wrist_example ${catkin_LIBRARIES})

## Test gripper
add_executable(gripper_control_example src/control_examples/gripper_control_example.cpp)
add_dependencies(gripper_control_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_control_example  ${catkin_LIBRARIES})


## interactive test
add_executable(interactive_arm_example src/control_examples/interactive_arm_example.cpp)
add_dependencies(interactive_arm_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_arm_example  ${catkin_LIBRARIES})

# ## nudge offset
# add_executable(nudge_offset_example src/control_examples/nudge_offset_example.cpp)
# add_dependencies(nudge_offset_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(nudge_offset_example  ${catkin_LIBRARIES})
