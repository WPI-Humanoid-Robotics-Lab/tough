cmake_minimum_required(VERSION 2.8.3)
project(tough_controller_interface)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ihmc_msgs
  roscpp
  std_msgs
  tf
  tough_common
)

# Added for unit testing
find_package(GTest REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_controller_interface
  CATKIN_DEPENDS geometry_msgs ihmc_msgs roscpp std_msgs tf tough_common
  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} include)

# Added for unit testing
include_directories(${GTEST_INCLUDE_DIRS})

#include_directories(
#  include /${PROJECT_NAME}
#  ${catkin_INCLUDE_DIRS} include
#  ${GTEST_INCLUDE_DIRS}
#)

 add_library(tough_controller_interface
   src/tough_controller_interface.cpp
   src/arm_control_interface.cpp
   src/pelvis_control_interface.cpp
   src/chest_control_interface.cpp
   src/head_control_interface.cpp
   src/gripper_control_interface.cpp
   src/wholebody_control_interface.cpp
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)


#add_executable(test_arm_navigation src/test_arm_navigation.cpp)
#add_dependencies(test_arm_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_arm_navigation  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(interactive_arm_test src/interactive_arm_test.cpp)
#add_dependencies(interactive_arm_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(interactive_arm_test  ${catkin_LIBRARIES} tough_controller_interface)

#add_executable(test_arm src/test_arm.cpp)
#add_dependencies(test_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_arm  ${catkin_LIBRARIES} ${PROJECT_NAME})


#add_executable(test_nudgeLocal src/test_nudgeLocal.cpp)
#add_dependencies(test_nudgeLocal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_nudgeLocal  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_chest src/test_chest_navigation.cpp)
#add_dependencies(test_chest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_chest  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_pelvis src/test_pelvis_navigation.cpp)
#add_dependencies(test_pelvis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_pelvis  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_head src/test_head_navigation.cpp)
#add_dependencies(test_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_head  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_neck src/test_neck_navigation.cpp)
#add_dependencies(test_neck ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_neck  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_wrist src/test_wrist.cpp)
#add_dependencies(test_wrist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_wrist ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_gripper src/test_gripper_control.cpp)
#add_dependencies(test_gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_gripper  ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(reset_robot src/reset_robot.cpp)
#add_dependencies(reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(reset_robot  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
#                test_chest
#                test_nudgeLocal
#                test_arm
#                test_arm_navigation
#                test_neck
#                test_head
#                test_pelvis
#                test_nudgeLocal
#                test_arm_navigation
#                test_wrist
#                test_gripper
#                reset_robot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )






# Added for unit testing

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test test/ros_gtest-test.cpp)
catkin_add_gtest(test_pelvis_height test/pelvis_control_interface_test.cpp)
catkin_add_gtest(test_arm_unit_test test/arm_control_interface_test.cpp)

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

target_link_libraries(test_pelvis_height ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(test_arm_unit_test ${catkin_LIBRARIES} ${PROJECT_NAME})
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

