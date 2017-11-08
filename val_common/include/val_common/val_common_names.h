#ifndef VAL_COMMON_NAMES_H
#define VAL_COMMON_NAMES_H

#include <string>

namespace VAL_COMMON_NAMES
{
/********* Supported Robots *********/
const std::string atlas = "atlas";
const std::string valkyrie = "valkyrie";

/********* Topic Names *********/
const std::string LOG_TOPIC = "/field/log";
const std::string MARKER_TOPIC = "/visualization_marker";
const std::string MARKER_ARRAY_TOPIC = "/visualization_marker_array";

/********* TF Frames *********/

/* Multisense */
const std::string HOKUYO_LINK_TF = "/hokuyo_link";
const std::string HEAD_HOKUYO_FRAME_TF = "/head_hokuyo_frame";
const std::string LEFT_CAMERA_OPTICAL_FRAME_TF = "/left_camera_optical_frame";
const std::string ROBOT_HEAD_FRAME_TF = "/head";

/* world frame*/
const std::string WORLD_TF = "/world";

/* Topics*/
const std::string RECTIFIED_IMAGE_TOPIC = "/multisense/camera/left/image_rect_color";

/* Moveit Groups */
// leftArm -- > leftShoulderPitchLink to leftPalm
const std::string LEFT_ARM_GROUP = "leftArm";

// rightArm -- > rightShoulderPitchLink to rightPalm
const std::string RIGHT_ARM_GROUP = "rightArm";

// leftPalm -- > pelvis to leftPalm
const std::string LEFT_PALM_GROUP = "leftPalm";

// rightPalm -- > pelvis to rightPalm
const std::string RIGHT_PALM_GROUP = "rightPalm";

// rightMiddleFingerGroup -- > pelvis to rightMiddleFingerPitch1Link
const std::string RIGHT_ENDEFFECTOR_GROUP = "rightMiddleFingerGroup";

// leftMiddleFingerGroup -- > pelvis to leftMiddleFingerPitch1Link
const std::string LEFT_ENDEFFECTOR_GROUP = "leftMiddleFingerGroup";

#ifdef ATLAS
//atlas Hand
const std::string R_PALM_TF = "/r_hand";
const std::string L_PALM_TF = "/l_hand";

// moveit is not configured for atlas yet!!!
///@todo configure moveit for atlas
const std::string R_END_EFFECTOR_FRAME = "/r_hand";
const std::string L_END_EFFECTOR_FRAME = "/l_hand";

/* atlas Torso*/
//const std::string PELVIS_TF = "/pelvis";
//const std::string TORSO_TF = "/mtorso";

/* atlas Right Foot*/
const std::string R_FOOT_TF = "/r_foot";

/* atlas Left Foot*/
const std::string L_FOOT_TF = "/l_foot";

/* Number of neck joint */
const int NUM_NECK_JOINTS = 1;
#else
// Build for Valkyrie by default
//valkyrie Hand
const std::string R_PALM_TF = "/rightPalm";
const std::string L_PALM_TF = "/leftPalm";

const std::string R_END_EFFECTOR_FRAME = "/rightMiddleFingerPitch1Link";
const std::string L_END_EFFECTOR_FRAME = "/leftMiddleFingerPitch1Link";

/* valkyrie Torso*/
//const std::string PELVIS_TF = "/pelvis";
//const std::string TORSO_TF = "/torso";

/* valkyrie Right Foot*/
const std::string R_FOOT_TF = "/rightFoot";

/* valkyrie Left Foot*/
const std::string L_FOOT_TF = "/leftFoot";


/* Number of neck joint */
const int NUM_NECK_JOINTS = 3;

#endif

}

#endif
