cmake_minimum_required(VERSION 2.8.3)
project(val_perception_bringup)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        geometric_shapes
                                        cv_bridge
                                        image_transport
                                        pcl_conversions
                                        pcl_ros
                                        message_filters
                                        multisense_ros
                                        tf_conversions
                                        laser_geometry
                                        laser_assembler
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        val_common
                                        val_controllers
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES val_perception_bringup
  CATKIN_DEPENDS roscpp cv_bridge image_transport pcl_conversions std_msgs message_filters multisense_ros laser_assembler val_common val_controllers
  DEPENDS opencv pcl
)

###########
## Build ##
###########
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable

 add_executable(left_image_inverter src/left_image_inverter.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
## set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(left_image_inverter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(left_image_inverter
   ${catkin_LIBRARIES}
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
