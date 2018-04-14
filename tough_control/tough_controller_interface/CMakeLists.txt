cmake_minimum_required(VERSION 2.8.3)
project(tough_controller_interface)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ihmc_msgs
  roscpp
  std_msgs
  tf
  tough_common
)
find_package(GTest REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tough_controller_interface
  CATKIN_DEPENDS geometry_msgs ihmc_msgs roscpp std_msgs tf tough_common
  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${GTEST_INCLUDE_DIRS})

 add_library(tough_controller_interface
   src/tough_controller_interface.cpp
   src/arm_control_interface.cpp
   src/pelvis_control_interface.cpp
   src/chest_control_interface.cpp
   src/head_control_interface.cpp
   src/gripper_control_interface.cpp
   src/wholebody_control_interface.cpp
)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)


add_executable(test_arm_navigation src/test_arm_navigation.cpp)
add_dependencies(test_arm_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm_navigation  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_arm src/test_arm.cpp)
add_dependencies(test_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_arm  ${catkin_LIBRARIES} ${PROJECT_NAME})


add_executable(test_nudgeLocal src/test_nudgeLocal.cpp)
add_dependencies(test_nudgeLocal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_nudgeLocal  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_chest src/test_chest_navigation.cpp)
add_dependencies(test_chest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_chest  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_pelvis src/test_pelvis_navigation.cpp)
add_dependencies(test_pelvis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pelvis  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_head src/test_head_navigation.cpp)
add_dependencies(test_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_head  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_neck src/test_neck_navigation.cpp)
add_dependencies(test_neck ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_neck  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_wrist src/test_wrist.cpp)
add_dependencies(test_wrist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_wrist ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_gripper src/test_gripper_control.cpp)
add_dependencies(test_gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gripper  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(reset_robot src/reset_robot.cpp)
add_dependencies(reset_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reset_robot  ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} test_chest test_nudgeLocal test_arm
                test_arm_navigation test_neck test_head test_pelvis
                test_chest test_nudgeLocal test_arm test_arm_navigation test_wrist
                test_gripper reset_robot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

add_executable(Test_Arms_And_Pelvis src/Examples/Test_Arms_And_Pelvis.cpp)
add_dependencies(Test_Arms_And_Pelvis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Test_Arms_And_Pelvis  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(Test_Head src/Examples/Test_Head.cpp)
add_dependencies(Test_Head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Test_Head  ${catkin_LIBRARIES} tough_controller_interface)

add_executable(Test_Gripper src/Examples/Test_Gripper.cpp)
add_dependencies(Test_Gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Test_Gripper  ${catkin_LIBRARIES} tough_controller_interface)

catkin_add_gtest(test_pelvis_height src/test/pelvis_control_interface_test.cpp)
target_link_libraries(test_pelvis_height ${catkin_LIBRARIES} tough_controller_interface)

catkin_add_gtest(test_chest_trajectory src/test/chest_control_interface_test.cpp)
target_link_libraries(test_chest_trajectory ${catkin_LIBRARIES} tough_controller_interface)

catkin_add_gtest(test_arms src/test/arms_control_interface_test.cpp)
target_link_libraries(test_arms ${catkin_LIBRARIES} tough_controller_interface)

catkin_add_gtest(test_head_case src/test/head_control_interface_test.cpp)
target_link_libraries(test_head_case ${catkin_LIBRARIES} tough_controller_interface)



