cmake_minimum_required(VERSION 2.8.3)
project(val_gazebo)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)
set(CMAKE_BUILD_TYPE DEBUG)

set(erb_models
  light_panel
)

add_custom_target(erb ALL)
foreach(model_folder_name ${erb_models})
  set(full_path ${CMAKE_CURRENT_SOURCE_DIR}/models/${model_folder_name})
  add_custom_command(TARGET erb
                     COMMAND erb model.erb > model.sdf
                     WORKING_DIRECTORY ${full_path}
                     COMMENT "generating model.sdf for ${model_folder_name}")
endforeach()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set (plugins_src
  LightControl
)

foreach(plugin_src ${plugins_src})
  add_library(${plugin_src} SHARED plugins/${plugin_src}.cc)
  target_link_libraries(${plugin_src} ${GAZEBO_LIBRARIES})
  target_link_libraries(${plugin_src} ${catkin_LIBRARIES})
endforeach()

