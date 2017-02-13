cmake_minimum_required(VERSION 2.8.3)
project(val_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ihmc_msgs
  roscpp
  std_msgs
  tf
  val_common
  val_footstep
)

#   std_msgs

catkin_package(
  INCLUDE_DIRS include LIBRARIES val_control CATKIN_DEPENDS geometry_msgs ihmc_msgs roscpp std_msgs tf val_common DEPENDS system_lib val_common val_footstep
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_library(val_control
   src/val_arm_navigation.cpp
   src/val_pelvis_navigation.cpp
   src/val_chest_navigation.cpp
)

 target_link_libraries(val_control
   ${catkin_LIBRARIES}
)

add_dependencies(val_control ${catkin_EXPORTED_TARGETS})
install(TARGETS val_control ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")