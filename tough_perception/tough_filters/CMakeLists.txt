cmake_minimum_required(VERSION 2.8.3)
project(tough_filters)

add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  robot_self_filter
  roscpp
  tf
  tough_common
  tough_perception_common
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tough_filters
  CATKIN_DEPENDS robot_self_filter roscpp tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} include)

add_executable(robot_filter src/robot_filter.cpp)
add_dependencies(robot_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_filter ${catkin_LIBRARIES} tough_common tough_perception_common)

add_executable(walkway_filter src/walkway_filter.cpp)
add_dependencies(walkway_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(walkway_filter ${catkin_LIBRARIES} tough_common tough_perception_common)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS walkway_filter robot_filter
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_filters.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
