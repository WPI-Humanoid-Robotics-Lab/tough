cmake_minimum_required(VERSION 2.8.3)
project(tough_perception_bringup)

find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)
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
                                        tough_common
                                        tough_controller_interface
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tough_perception_bringup
  CATKIN_DEPENDS roscpp cv_bridge image_transport pcl_conversions std_msgs message_filters multisense_ros laser_assembler tough_common tough_controller_interface
  DEPENDS opencv pcl
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
