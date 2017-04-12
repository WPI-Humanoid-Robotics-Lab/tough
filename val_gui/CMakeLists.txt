cmake_minimum_required(VERSION 2.8.3)
project(val_gui)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

add_definitions(-std=c++11)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
add_definitions(${QT_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  val_control
  val_footstep
)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
set(HEADER_FILES
  include/${PROJECT_NAME}/val_gui.h
  include/${PROJECT_NAME}/configurationreader.h
)

set(SOURCE_FILES
  src/main.cpp
  src/val_gui.cpp
  src/configurationreader.cpp
  ${MOC_FILES}
)


set(FORM_FILES
  src/val_gui.ui
)


QT4_WRAP_CPP(MOC_SRC_H
  ${HEADER_FILES}
)

QT4_WRAP_UI(MOC_GUI_H
  ${FORM_FILES}
)

add_library(val_marker
  src/val_marker.cpp
)

target_link_libraries(val_marker
  ${catkin_LIBRARIES}
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES val_marker
  CATKIN_DEPENDS roscpp rviz sensor_msgs std_msgs val_control val_footstep
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(val_gui ${SOURCE_FILES} ${MOC_SRC_H} ${MOC_GUI_H})
target_link_libraries(val_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/val_gui_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_gui.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
