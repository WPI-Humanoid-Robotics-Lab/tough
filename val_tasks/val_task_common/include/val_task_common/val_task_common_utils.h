#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#define DISTANCE_TOLERANCE 0.02      // 2cm
#define ANGLE_TOLERANCE    0.0174533   // 1 degree

namespace taskCommonUtils {

bool isPoseChanged(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new);

}
