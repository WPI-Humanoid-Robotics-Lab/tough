#include <val_task_common/val_task_common_utils.h>

bool taskCommonUtils::isPoseChanged(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new)
{
    bool ret = false;
    if (sqrt(pow((pose_new.y - pose_old.y),2) + pow((pose_new.x - pose_old.x),2)) > DISTANCE_TOLERANCE)
    {
        ret = true;
    }
    else if (fabs(fmod(pose_new.theta,(2*M_PI)) - fmod(pose_new.theta,(2*M_PI))) > ANGLE_TOLERANCE)
    {
        ret = true;
    }

    return ret;
}
