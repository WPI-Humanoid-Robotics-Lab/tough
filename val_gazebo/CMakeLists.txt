cmake_minimum_required(VERSION 2.8.3)
project(val_gazebo)

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

  #add_dependencies(erb ${full_path}/model.sdf)
endforeach()

