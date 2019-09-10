cmake_minimum_required(VERSION 2.8.3)
project(tough_kinematics)

add_definitions(-std=c++11)

find_package(catkin REQUIRED tough_common trac_ik_lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_kinematics
  CATKIN_DEPENDS tough_common trac_ik_lib
#  DEPENDS system_lib
)

###########
## Build ##
###########

 include_directories(${catkin_INCLUDE_DIRS} include)

 add_library(${PROJECT_NAME}
   src/tough_kinematics.cpp
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tough_kinematics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
