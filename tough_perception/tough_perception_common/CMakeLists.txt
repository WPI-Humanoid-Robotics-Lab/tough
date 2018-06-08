cmake_minimum_required(VERSION 2.8.3)
project(tough_perception_common)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")

find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)
find_package(PCL 1.7 REQUIRED io)
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

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp cv_bridge image_transport pcl_conversions std_msgs message_filters multisense_ros laser_assembler tough_common tough_controller_interface
   DEPENDS opencv pcl
)

add_library(${PROJECT_NAME}  src/MultisensePointCloud.cpp
                             src/MultisenseImage.cpp
#                             src/ImageHelper.cpp
                             src/PointCloudHelper.cpp
                             src/laser2point_cloud.cpp
)
									
set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_DEFINITIONS GAZEBO_SIMULATION)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(test_lasercloud  src/test_laser.cpp)
target_link_libraries(test_lasercloud  ${PROJECT_NAME})

#add_executable(test_image  src/test_image.cpp)
#target_link_libraries(test_image  ${PROJECT_NAME})

add_executable(test_organizedRGBD  src/test_organizedRGBD.cpp)
target_link_libraries(test_organizedRGBD  ${PROJECT_NAME})

add_executable(periodic_snapshotter  src/periodic_snapshotter.cpp)
target_link_libraries(periodic_snapshotter  ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARY_DIRS})

add_executable(walkway_point_generator src/walkway_generator.cpp)
add_dependencies(walkway_point_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(walkway_point_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

 add_executable(laser2point_cloud_node src/laser2point_cloud_node.cpp)
 add_dependencies(laser2point_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(laser2point_cloud_node ${catkin_LIBRARIES} ${PROJECT_NAME} )


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

