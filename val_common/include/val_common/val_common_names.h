#ifndef VAL_COMMON_NAMES_H
#define VAL_COMMON_NAMES_H

#include <string>

namespace VAL_COMMON_NAMES
{
    /********* TF Frames *********/

    /* Multisense */
    static const std::string HOKUYO_LINK_TF = "/hokuyo_link";
    static const std::string HEAD_HOKUYO_FRAME_TF = "/head_hokuyo_frame";
    static const std::string LEFT_CAMERA_OPTICAL_FRAME_TF = "/left_camera_optical_frame";
    static const std::string ROBOT_HEAD_FRAME_TF = "/head";

    //valkyrie Hand
    static const std::string R_PALM_TF = "/rightPalm";
    static const std::string L_PALM_TF = "/leftPalm";

    /* valkyrie Torso*/
    static const std::string PELVIS_TF = "/pelvis";
    static const std::string TORSO_TF = "/torso";

    /* valkyrie Right Foot*/
    static const std::string R_FOOT_TF = "/rightFoot";

    /* valkyrie Left Foot*/
    static const std::string L_FOOT_TF = "/leftFoot";

    /* world frame*/
    static const std::string WORLD_TF = "/world";

    /* Topics*/
    static const std::string RECTIFIED_IMAGE_TOPIC = "/multisense/camera/left/image_rect_color";

}

#endif
