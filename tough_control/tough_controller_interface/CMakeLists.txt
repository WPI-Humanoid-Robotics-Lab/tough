cmake_minimum_required(VERSION 2.8.3)
project(tough_controller_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ihmc_msgs
  roscpp
  std_msgs
  tf
  tough_common
)

# Added for unit testing
#find_package(GTest REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_controller_interface
  CATKIN_DEPENDS geometry_msgs ihmc_msgs roscpp std_msgs tf tough_common
  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} include)

# Added for unit testing
#include_directories(${GTEST_INCLUDE_DIRS})

#include_directories(
#  include /${PROJECT_NAME}
#  ${catkin_INCLUDE_DIRS} include
#  ${GTEST_INCLUDE_DIRS}
#)

 add_library(tough_controller_interface
   src/tough_control_interface.cpp
   src/arm_control_interface.cpp
   src/pelvis_control_interface.cpp
   src/chest_control_interface.cpp
   src/head_control_interface.cpp
   src/gripper_control_interface.cpp
   src/wholebody_control_interface.cpp
   src/leg_control_interface.cpp
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)



add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
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
#catkin_add_gtest(test_pelvis_height test/pelvis_control_interface_test.cpp)
#catkin_add_gtest(test_arm_unit_test test/arm_control_interface_test.cpp)

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

#target_link_libraries(test_pelvis_height ${catkin_LIBRARIES} ${PROJECT_NAME})
#target_link_libraries(test_arm_unit_test ${catkin_LIBRARIES} ${PROJECT_NAME})
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

