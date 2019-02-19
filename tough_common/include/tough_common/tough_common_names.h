#ifndef VAL_COMMON_NAMES_H
#define VAL_COMMON_NAMES_H

#include <string>

namespace TOUGH_COMMON_NAMES
{
/********* Supported Robots *********/
const std::string atlas = "atlas";
const std::string valkyrie = "valkyrie";

/********* Parameters *********/
const std::string ROBOT_NAME_PARAM = "/ihmc_ros/robot_name";
const std::string LEFT_ARM_JOINT_NAMES_PARAM = "left_arm_joint_names";
const std::string RIGHT_ARM_JOINT_NAMES_PARAM = "right_arm_joint_names";
const std::string CHEST_JOINT_NAMES_PARAM = "chest_joint_names";
const std::string LEFT_FOOT_FRAME_NAME_PARAM = "left_foot_frame_name";
const std::string RIGHT_FOOT_FRAME_NAME_PARAM = "right_foot_frame_name";
const std::string LEFT_EE_FRAME_NAME_PARAM = "left_ee_frame_name";
const std::string RIGHT_EE_FRAME_NAME_PARAM = "right_ee_frame_name";

/********* Topic Names *********/
const std::string TOPIC_PREFIX = "/ihmc_ros/";
const std::string LOG_TOPIC = "/field/log";
const std::string MARKER_TOPIC = "/visualization_marker";
const std::string MARKER_ARRAY_TOPIC = "/visualization_marker_array";
const std::string RECTIFIED_IMAGE_TOPIC = "/multisense/camera/left/image_rect_color";

/********* Control Topics *********/
const std::string CONTROL_TOPIC_PREFIX = "/control";
const std::string ARM_TRAJECTORY_TOPIC = "/arm_trajectory";
const std::string HAND_TRAJECTORY_TOPIC = "/hand_trajectory";
const std::string CHEST_TRAJECTORY_TOPIC = "/chest_trajectory";
const std::string HEAD_TRAJECTORY_TOPIC = "/head_trajectory";
const std::string NECK_TRAJECTORY_TOPIC = "/neck_trajectory";
const std::string PELVIS_HEIGHT_TRAJECTORY_TOPIC = "/pelvis_height_trajectory";
const std::string WHOLEBODY_TRAJECTORY_TOPIC = "/whole_body_trajectory";
const std::string STOP_ALL_TRAJECTORY_TOPIC = "/stop_all_trajectories";
const std::string HAND_DESIRED_CONFIG_TOPIC = "/hand_desired_configuration";
const std::string GO_HOME_TOPIC = "/go_home";

/********* Output Topics *********/
const std::string OUTPUT_TOPIC_PREFIX = "/output";
const std::string JOINT_STATES_TOPIC = "/joint_states";

/********* Frame Hash from IHMC controllers *********/
const int MIDFEET_ZUP_FRAME_HASH = -100;
const int PELVIS_ZUP_FRAME_HASH = -101;
const int PELVIS_FRAME_HASH = -102;
const int CHEST_FRAME_HASH = -103;
const int CENTER_OF_MASS_FRAME_HASH = -104;
const int LEFT_SOLE_FRAME_HASH = -105;
const int RIGHT_SOLE_FRAME_HASH = -106;
const int WORLD_FRAME_HASH = -107;
/********* TF Frames *********/

/* Multisense */
const std::string HOKUYO_LINK_TF = "/multisense/hokuyo_link";
const std::string HEAD_HOKUYO_FRAME_TF = "/multisense/head_hokuyo_frame";
const std::string LEFT_CAMERA_OPTICAL_FRAME_TF = "/multisense/left_camera_optical_frame";
const std::string ROBOT_HEAD_FRAME_TF = "/head";

/* world frame*/
const std::string WORLD_TF = "/world";

/********* MoveIt params *********/
/* Moveit Groups */
const std::string LEFT_ARM_7DOF_GROUP = "L_SHOULDER_HAND_7DOF";
const std::string RIGHT_ARM_7DOF_GROUP = "R_SHOULDER_HAND_7DOF";

const std::string LEFT_ARM_10DOF_GROUP = "L_PELVIS_PALM_10DOF";
const std::string RIGHT_ARM_10DOF_GROUP = "R_PELVIS_PALM_10DOF";

const std::string PLANNING_PLUGIN_PARAM = "/move_group/planning_plugin";

const std::string TRAJECTORY_DISPLAY_TOPIC = "/move_group/display_planned_path";

/********* Footstep Planning *********/
const std::string FOOTSTEP_PLANNER_SERVICE = "/plan_footsteps";

}  // namespace TOUGH_COMMON_NAMES

#endif
