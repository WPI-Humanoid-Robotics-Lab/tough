cmake_minimum_required(VERSION 2.8.3)
project(led_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  perception_common
  val_common
)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES led_detector
  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp sensor_msgs std_msgs val_common perception_common
  DEPENDS system_lib
  # INCLUDE_DIRS include
  # LIBRARIES cv_image_warp cv_line_tools cv_pca #?

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(perception_common
   ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library
#add_library(led_detector
#  src/LedDetector.cpp
#)
#target_link_libraries(led_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(led_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(led_detector src/LedDetector.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(val_led_pos_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(led_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} perception_common
)
