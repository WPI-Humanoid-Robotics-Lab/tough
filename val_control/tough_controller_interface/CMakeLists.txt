cmake_minimum_required(VERSION 2.8.3)
project(tough_controller_interface)

add_definitions(-D$ENV{ROBOT_NAME})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ihmc_msgs
  roscpp
  std_msgs
  tf
  val_common
)

#   std_msgs

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_controller_interface
  CATKIN_DEPENDS geometry_msgs ihmc_msgs roscpp std_msgs tf val_common
  DEPENDS system_lib val_common val_footstep
)

include_directories(${catkin_INCLUDE_DIRS} include)

 add_library(tough_controller_interface
   src/arm_control_interface.cpp
   src/pelvis_control_interface.cpp
   src/chest_control_interface.cpp
   src/head_control_interface.cpp
   src/gripper_control_interface.cpp
   src/robot_state.cpp
   src/wholebody_control_interface.cpp
)

 target_link_libraries(tough_controller_interface
   ${catkin_LIBRARIES}
)


add_executable(test_jointState src/test_robot_state.cpp)
add_dependencies(test_jointState  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_jointState   ${catkin_LIBRARIES} tough_controller_interface)


add_executable(test_arm_navigation src/test_arm_navigation.cpp)
add_dependencies(test_arm_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm_navigation  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_arm src/test_arm.cpp)
add_dependencies(test_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm  ${catkin_LIBRARIES} tough_controller_interface)


add_executable(test_nudgeLocal src/test_nudgeLocal.cpp)
add_dependencies(test_nudgeLocal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_nudgeLocal  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_chest src/test_chest_navigation.cpp)
add_dependencies(test_chest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_chest  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_pelvis src/test_pelvis_navigation.cpp)
add_dependencies(test_pelvis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pelvis  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_head src/test_head_navigation.cpp)
add_dependencies(test_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_head  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_neck src/test_neck_navigation.cpp)
add_dependencies(test_neck ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_neck  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_wrist src/test_wrist.cpp)
add_dependencies(test_wrist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_wrist ${catkin_LIBRARIES} tough_controller_interface)

add_executable(test_gripper src/test_gripper_control.cpp)
add_dependencies(test_gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gripper  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(reset_robot src/reset_robot.cpp)
add_dependencies(reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reset_robot  ${catkin_LIBRARIES} tough_controller_interface)

add_dependencies(tough_controller_interface ${catkin_EXPORTED_TARGETS})
install(TARGETS tough_controller_interface ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")
