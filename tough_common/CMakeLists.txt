cmake_minimum_required(VERSION 2.8.3)
project(tough_common)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp urdf)

#kdtree is not required here. This should be removed.
set(KDSourcesList src/kdtree/alglibinternal.cpp src/kdtree/ap.cpp src/kdtree/alglibmisc.cpp)

set(HEADERS
    include/${PROJECT_NAME}/robot_description.h)

set(SOURCES
    src/robot_description.cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_common
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS} include)

## Declare a C++ library
add_library(kdtree
   ${KDSourcesList}
 )

add_library(tough_common ${HEADERS} ${SOURCES})
set_target_properties(tough_common PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(tough_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tough_common ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(robot_description src/robot_description.cpp)
add_dependencies(robot_description ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_description   ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS kdtree tough_common robot_description
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
