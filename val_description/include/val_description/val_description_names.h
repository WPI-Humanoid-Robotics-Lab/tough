#ifndef VAL_DESCRIPTION_NAMES_H
#define VAL_DESCRIPTION_NAMES_H

#include <string>

namespace DESCRIPTION_COMMON_NAMES
{
    /********* TF Frames *********/

    /* Multisense */
    static const std::string HOKUYO_LINK_TF = "/hokuyo_link";
    static const std::string HEAD_HOKUYO_FRAME_TF = "/head_hokuyo_frame";
    static const std::string LEFT_CAMERA_OPTICAL_FRAME_TF = "/left_camera_frame";

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
}

#endif
