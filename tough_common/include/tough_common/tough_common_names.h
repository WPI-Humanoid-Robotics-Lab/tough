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
const std::string LEFT_ARM_JOINT_NAMES_PARAM  = "left_arm_joint_names" ;
const std::string RIGHT_ARM_JOINT_NAMES_PARAM = "right_arm_joint_names";
const std::string CHEST_JOINT_NAMES_PARAM     = "chest_joint_names";
const std::string LEFT_FOOT_FRAME_NAME_PARAM  = "left_foot_frame_name" ;
const std::string RIGHT_FOOT_FRAME_NAME_PARAM = "right_foot_frame_name";


/********* Topic Names *********/
const std::string LOG_TOPIC = "/field/log";
const std::string MARKER_TOPIC = "/visualization_marker";
const std::string MARKER_ARRAY_TOPIC = "/visualization_marker_array";
const std::string JOINT_STATES_TOPIC = "/joint_states";
const std::string RECTIFIED_IMAGE_TOPIC = "/multisense/camera/left/image_rect_color";

/********* TF Frames *********/

/* Multisense */
const std::string HOKUYO_LINK_TF = "/multisense/hokuyo_link";
const std::string HEAD_HOKUYO_FRAME_TF = "/multisense/head_hokuyo_frame";
const std::string LEFT_CAMERA_OPTICAL_FRAME_TF = "/multisense/left_camera_optical_frame";
const std::string ROBOT_HEAD_FRAME_TF = "/head";

/* world frame*/
const std::string WORLD_TF = "/world";


/* Moveit Groups */
// leftArm -- > leftShoulderPitchLink to leftPalm
const std::string LEFT_ARM_7DOF_GROUP = "L_SHOULDER_HAND_7DOF";

// rightArm -- > rightShoulderPitchLink to rightPalm
const std::string RIGHT_ARM_7DOF_GROUP = "R_SHOULDER_HAND_7DOF";

// leftArm -- > leftEndEffectorFrame wrt leftHand/leftPalm
const std::string LEFT_END_EFFECTOR_FRAME = "l_palm";

// rightArm -- > rightEndEffectorFrame wrt rightHand/rightPalm
const std::string RIGHT_END_EFFECTOR_FRAME = "r_palm";

//// leftPalm -- > pelvis to leftPalm
//const std::string LEFT_PALM_GROUP = "leftPalm";

//// rightPalm -- > pelvis to rightPalm
//const std::string RIGHT_PALM_GROUP = "rightPalm";

// rightMiddleFingerGroup -- > pelvis to rightMiddleFingerPitch1Link
const std::string RIGHT_ARM_10DOF_GROUP = "R_PELVIS_PALM_10DOF";

// leftMiddleFingerGroup -- > pelvis to leftMiddleFingerPitch1Link
const std::string LEFT_ARM_10DOF_GROUP = "L_PELVIS_PALM_10DOF";


}

#endif
