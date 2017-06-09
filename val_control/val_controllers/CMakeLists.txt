cmake_minimum_required(VERSION 2.8.3)
project(val_controllers)

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
  LIBRARIES val_controllers
  CATKIN_DEPENDS geometry_msgs ihmc_msgs roscpp std_msgs tf val_common
  DEPENDS system_lib val_common val_footstep
)

include_directories(${catkin_INCLUDE_DIRS} include)

 add_library(val_controllers
   src/val_arm_navigation.cpp
   src/val_pelvis_navigation.cpp
   src/val_chest_navigation.cpp
   src/val_head_navigation.cpp
   src/val_gripper_control.cpp
   src/robot_state.cpp
   src/val_wholebody_manipulation.cpp
)

 target_link_libraries(val_controllers
   ${catkin_LIBRARIES}
)


add_executable(test_jointState src/test_robot_state.cpp)
add_dependencies(test_jointState  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_jointState   ${catkin_LIBRARIES} val_controllers)

add_executable(test_arm src/test_arm_navigation.cpp)
add_dependencies(test_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm  ${catkin_LIBRARIES} val_controllers)

add_executable(test_nudgeLocal src/test_nudgeLocal.cpp)
add_dependencies(test_nudgeLocal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_nudgeLocal  ${catkin_LIBRARIES} val_controllers)

add_executable(test_chest src/test_chest_navigation.cpp)
add_dependencies(test_chest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_chest  ${catkin_LIBRARIES} val_controllers)

add_executable(test_pelvis src/test_pelvis_navigation.cpp)
add_dependencies(test_pelvis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pelvis  ${catkin_LIBRARIES} val_controllers)

add_executable(test_head src/test_head_navigation.cpp)
add_dependencies(test_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_head  ${catkin_LIBRARIES} val_controllers)

add_executable(test_neck src/test_neck_navigation.cpp)
add_dependencies(test_neck ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_neck  ${catkin_LIBRARIES} val_controllers)

add_executable(test_gripper src/test_gripper_control.cpp)
add_dependencies(test_gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gripper  ${catkin_LIBRARIES} val_controllers)

add_executable(reset_robot src/reset_robot.cpp)
add_dependencies(reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reset_robot  ${catkin_LIBRARIES} val_controllers)

add_dependencies(val_controllers ${catkin_EXPORTED_TARGETS})
install(TARGETS val_controllers ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")
