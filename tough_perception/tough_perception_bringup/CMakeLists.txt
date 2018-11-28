cmake_minimum_required(VERSION 2.8.3)
project(tough_perception_bringup)

find_package(catkin REQUIRED COMPONENTS tough_filters tough_perception_common )

#find_package(Qt5Gui)
#find_package(Qt5Core)
#find_package(Qt5Widgets)
#find_package(Qt5Test)
#find_package(Qt5Concurrent)
#find_package(Qt5OpenGL)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS tough_filters tough_perception_common
#  DEPENDS opencv pcl
)

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_perception_bringup.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
