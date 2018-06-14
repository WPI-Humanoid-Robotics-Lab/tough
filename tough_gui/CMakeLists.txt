cmake_minimum_required(VERSION 2.8.3)
project(tough_gui)

find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tough_controller_interface
  tough_moveit_planners
  moveit_ros_visualization
  tough_footstep
)

set(HEADER_FILES
  include/${PROJECT_NAME}/tough_gui.h
  include/${PROJECT_NAME}/configurationreader.h
)

set(SOURCE_FILES
  src/main.cpp
  src/tough_gui.cpp
  src/configurationreader.cpp
)


set(FORM_FILES
  src/tough_gui.ui
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rviz sensor_msgs std_msgs tough_controller_interface tough_footstep
  DEPENDS system_lib
)


QT5_WRAP_CPP(MOC_SRC_H
  ${HEADER_FILES}
)

QT5_WRAP_UI(MOC_GUI_H
  ${FORM_FILES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_SRC_H} ${MOC_GUI_H})
target_link_libraries(${PROJECT_NAME} ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES})
qt5_use_modules(${PROJECT_NAME} Widgets)


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
