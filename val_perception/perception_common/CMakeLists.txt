cmake_minimum_required(VERSION 2.8.3)
project(perception_common)

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
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        nav_msgs
)

include_directories(include)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp cv_bridge image_transport pcl_conversions std_msgs message_filters multisense_ros 
   DEPENDS opencv pcl
)

add_library(${PROJECT_NAME}  src/MultisensePointCloud.cpp
                             src/MultisenseImage.cpp
                             src/ImageHelper.cpp
                             src/PointCloudHelper.cpp
                             src/laser2point_cloud.cpp
)
									
# TODO
set_property(
                TARGET ${PROJECT_NAME}
   		PROPERTY COMPILE_DEFINITIONS GAZEBO_SIMULATION)

target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})

add_executable(publish_corrected_map  src/publish_corrected_map.cpp)
target_link_libraries(publish_corrected_map  ${catkin_LIBRARIES})

add_executable(test_lasercloud  src/test_laser.cpp)
target_link_libraries(test_lasercloud  ${PROJECT_NAME})

add_executable(test_image  src/test_image.cpp)
target_link_libraries(test_image  ${PROJECT_NAME})

add_executable(test_organizedRGBD  src/test_organizedRGBD.cpp)
target_link_libraries(test_organizedRGBD  ${PROJECT_NAME})

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} multisense_backend
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
