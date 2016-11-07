cmake_minimum_required(VERSION 2.8.3)
project(srcsim)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)


########################
## Message generation ##
########################

add_message_files(
  FILES
  Console.msg
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)

##########################
## Multisense SL        ##
##########################

# Create the libSRCMultisensPlugin.so library.
set(src_multisense_sl_plugin_name SRCMultiSenseSLPlugin)
add_library(${src_multisense_sl_plugin_name} src/SRCMultiSenseSLPlugin.cc)
target_link_libraries(${src_multisense_sl_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
install(TARGETS ${src_multisense_sl_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

##########################
## Qualification task 1 ##
##########################

# Create the libQual1Plugin.so library.
set(qual1_plugin_name Qual1Plugin)
add_library(${qual1_plugin_name} src/Qual1Plugin.cc)
target_link_libraries(${qual1_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

add_dependencies(${qual1_plugin_name} srcsim_generate_messages_cpp)

install(TARGETS ${qual1_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

##########################
## Qualification task 2 ##
##########################

# Create the libQual2Plugin.so library.
set(qual2_plugin_name Qual2Plugin)
add_library(${qual2_plugin_name} src/Qual2Plugin.cc)
target_link_libraries(${qual2_plugin_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
#add_dependencies(${qual2_plugin_name}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)
install(TARGETS ${qual2_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


#############
## Install ##
#############

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install Misc. Scripts
install(PROGRAMS scripts/init_robot.sh scripts/walk_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
