#include <val_task_common/val_task_common_utils.h>
#include <tf/tf.h>

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


bool taskCommonUtils::isGoalReached(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new)
{
//    return !taskCommonUtils::isPoseChanged(pose_old, pose_new);
    // cannot reuse isPoseChanged, need a larger tolerance for goal
    bool ret = true;
    if (sqrt(pow((pose_new.y - pose_old.y),2) + pow((pose_new.x - pose_old.x),2)) > GOAL_DISTANCE_TOLERANCE)
    {
        ret = false;
    }
    else if (fabs(fmod(pose_new.theta,(2*M_PI)) - fmod(pose_new.theta,(2*M_PI))) > GOAL_ANGLE_TOLERANCE)
    {
        ret = false;
    }

    return ret;
}


bool taskCommonUtils::isGoalReached(geometry_msgs::Pose pose_old, geometry_msgs::Pose2D pose_new)
{
    geometry_msgs::Pose2D pose2d_old;
    pose2d_old.x = pose_old.position.x;
    pose2d_old.y = pose_old.position.y;
    pose2d_old.theta = tf::getYaw(pose_old.orientation);
    return !taskCommonUtils::isPoseChanged(pose2d_old, pose_new);
}
