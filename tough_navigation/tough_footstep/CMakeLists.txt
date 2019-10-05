cmake_minimum_required(VERSION 2.8.3)
project(tough_footstep)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  footstep_planner
  gridmap_2d
  humanoid_localization
  humanoid_nav_msgs 
  tf
  angles
  navigation_common
  message_generation
  actionlib
  std_msgs
  geometry_msgs
  visualization_msgs
  ihmc_msgs
  tough_common
  tough_controller_interface
  )
find_package(PkgConfig REQUIRED)

pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

 add_message_files(DIRECTORY msg
  FILES
   StepTargetArray.msg
  )
### Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   humanoid_nav_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime humanoid_nav_msgs tough_common
  DEPENDS SBPL
  )

###########
## Build ##
###########
include_directories( ${catkin_INCLUDE_DIRS} ${SBPL_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME}
   src/robot_walker.cpp
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

add_executable(footstep_node src/footstep_node.cpp)
target_link_libraries(footstep_node ${catkin_LIBRARIES} ${PROJECT_NAME} )

# add_executable(test_footstep src/test_footstep.cpp)
# add_dependencies(test_footstep ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_footstep  ${catkin_LIBRARIES})



#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} 
    footstep_node
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

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.yaml"
  PATTERN "*.rviz"
)

install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.yaml"
  PATTERN "*.pgm"
  PATTERN "*.gif"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_footstep.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
