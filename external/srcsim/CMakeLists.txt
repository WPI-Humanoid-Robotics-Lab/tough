cmake_minimum_required(VERSION 2.8.3)
project(srcsim)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

#####################
## Find components ##
#####################
set (COMPONENT_LIST std_msgs message_generation)

if(NOT BUILD_MSGS_ONLY)
  list (APPEND COMPONENT_LIST gazebo_plugins gazebo_ros roscpp)
endif()

find_package(catkin REQUIRED COMPONENTS ${COMPONENT_LIST})

set (SRCSIM_MAJOR_VERSION 0)
set (SRCSIM_MINOR_VERSION 5)
set (SRCSIM_PATCH_VERSION 0)
set (SRCSIM_VERSION_FULL ${SRCSIM_MAJOR_VERSION}.${SRCSIM_MINOR_VERSION}.${SRCSIM_PATCH_VERSION})

if(NOT BUILD_MSGS_ONLY)
  find_package(gazebo REQUIRED)
endif()

########################
## Message generation ##
########################

add_message_files(
  FILES
  Console.msg
  Leak.msg
  Satellite.msg
  Score.msg
  Task.msg
)

add_service_files(
  FILES
  StartTask.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

if(NOT BUILD_MSGS_ONLY)
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

  ##########################
  ##         Finals       ##
  ##########################

  # Create the libFinalsPlugin.so library.
  set(finals_plugin_name FinalsPlugin)
  add_library(${finals_plugin_name} SHARED
    src/FinalsPlugin.cc
    src/Task.cc
    src/Task1.cc
    src/Task2.cc
    src/Task3.cc
    src/Checkpoint.cc
  )
  target_link_libraries(${finals_plugin_name}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
  )
  add_dependencies(${finals_plugin_name} srcsim_generate_messages_cpp)
  install(TARGETS ${finals_plugin_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  ##########################
  ## Box plugin ##
  ##########################

  # Create the libBoxContainsPlugin.so library.
  set(box_plugin_name BoxContainsPlugin)
  add_library(${box_plugin_name} SHARED
    src/BoxContainsPlugin.cc
  )
  target_link_libraries(${box_plugin_name}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
  )
  install(TARGETS ${box_plugin_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  ##########################
  ## Satellite plugin ##
  ##########################

  # Create the libSatellitePlugin.so library.
  set(satellite_plugin_name SatellitePlugin)
  add_library(${satellite_plugin_name} SHARED
    src/SatellitePlugin.cc
  )
  target_link_libraries(${satellite_plugin_name}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
  )

  add_dependencies(${satellite_plugin_name} srcsim_generate_messages_cpp)

  install(TARGETS ${satellite_plugin_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  ##########################
  ##  Solar Panel plugin  ##
  ##########################

  # Create the libSolarPanelPlugin.so library.
  set(solar_panel_plugin_name SolarPanelPlugin)
  add_library(${solar_panel_plugin_name} SHARED
    src/SolarPanelPlugin.cc
  )
  target_link_libraries(${solar_panel_plugin_name}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
  )

  install(TARGETS ${solar_panel_plugin_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  ##########################
  ##  SRC Harness plugin  ##
  ##########################

  # Create the libSRCHarnessPlugin.so library.
  set(src_harness_plugin_name SRCHarnessPlugin)
  add_library(${src_harness_plugin_name} SHARED
    src/SRCHarnessPlugin.cc
  )
  target_link_libraries(${src_harness_plugin_name}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
  )

  install(TARGETS ${src_harness_plugin_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  ##############################
  ##  SRC Ros Harness plugin  ##
  ##############################

  # Create the libSRCHarnessPlugin.so library.
  set(src_ros_harness_plugin_name srcsim_ros_harness)
  add_library(${src_ros_harness_plugin_name} SHARED
    src/srcsim_ros_harness.cpp
  )
  target_link_libraries(${src_ros_harness_plugin_name}
    ${catkin_LIBRARIES}
    ${src_harness_plugin_name}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
  )

  install(TARGETS ${src_ros_harness_plugin_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  #############
  ## Install ##
  #############

  # Install all the config files
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

  # Install all the launch files
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  # Install all the world files
  install(DIRECTORY worlds/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

  # Install Misc. Scripts
  install(PROGRAMS scripts/init_robot.sh scripts/walk_test.py scripts/rossleep.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  # Install all the model files
  install(DIRECTORY models/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

  # Configure the setup script
  catkin_add_env_hooks(99_srcsim_setup
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    SHELLS sh)
  # Configure the setup.bash system script
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/setup.bash.in
    ${PROJECT_BINARY_DIR}/setup.bash @ONLY)
  install(FILES ${PROJECT_BINARY_DIR}/setup.bash
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  # Packaging configuration
  set (CPACK_PACKAGE_VERSION "${DRC_SIM_VERSION_FULL}")
  set (CPACK_PACKAGE_VERSION_MAJOR "${SRCSIM_MAJOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_MINOR "${SRCSIM_MINOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_PATCH "${SRCSIM_PATCH_VERSION}")
  list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
  list (APPEND CPACK_SOURCE_IGNORE_FILES ";/.hg/;.hgignore;.swp$;/build/")
  set (CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${SRCSIM_VERSION_FULL}")
  include (CPack)
endif()  # BUILD_MSGS_ONLY
