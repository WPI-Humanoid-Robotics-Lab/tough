cmake_minimum_required(VERSION 2.8.3)
project(val_manipulation)

find_package(catkin REQUIRED COMPONENTS
  #  orocos_kdl

  tf
  trac_ik_lib
  val_common
  val_footstep
)

find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS tf trac_ik_lib DEPENDS system_lib orocos_kdl val_common val_footstep
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/val_manipulation.cpp
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(val_ik_solver_node
  src/val_ik_solver.cpp
)

add_dependencies(val_ik_solver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

 target_link_libraries(val_ik_solver_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
)

install(TARGETS val_ik_solver_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
