cmake_minimum_required(VERSION 2.8.3)
project(find_button)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  perception_common
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
 INCLUDE_DIRS include
  LIBRARIES find_button
  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(perception_common
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(find_button
   src/ButtonDetector.cpp
   

 )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_find_button.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(PCLImage_test src/PCL2Image_test.cpp)
target_link_libraries(PCLImage_test ${catkin_LIBRARIES})

add_executable(ButtonDectorTest src/ButtonDectorTest.cpp)
target_link_libraries(ButtonDectorTest ${catkin_LIBRARIES})