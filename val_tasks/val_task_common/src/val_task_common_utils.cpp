#include <val_task_common/val_task_common_utils.h>
#include <tf/tf.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_controller_interface/head_control_interface.h>
#include <tough_controller_interface/arm_control_interface.h>

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
//        ROS_WARN("taskCommonUtils::isGoalReached : Position Tolerance exceeded");
        ret = false;
    }

    else if (fabs(fmod(pose_new.theta,(2*M_PI)) - fmod(pose_new.theta,(2*M_PI))) > GOAL_ANGLE_TOLERANCE)
    {
        ROS_WARN("taskCommonUtils::isGoalReached : Angle Tolerance exceeded");
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
    return taskCommonUtils::isGoalReached(pose2d_old, pose_new);
}

void taskCommonUtils::moveToWalkSafePose(ros::NodeHandle &nh)
{
    ROS_INFO("Moving to walk safe pose");
    chestTrajectory chest_controller(nh);
    chest_controller.controlChest(0.0f, 0.0f, 0.0f);
    ros::Duration(0.5).sleep();

    pelvisTrajectory pelvis_controller(nh);
    pelvis_controller.controlPelvisHeight(0.9f);
    ros::Duration(0.5).sleep();

    HeadTrajectory  head_controller(nh);
    head_controller.moveHead(0.0f, 0.0f, 0.0f);
    ros::Duration(1).sleep();

    gripperControl gripper_controller(nh);
    gripper_controller.openGripper(armSide::RIGHT);
    gripper_controller.openGripper(armSide::LEFT);
    ros::Duration(0.2).sleep();

    armTrajectory arm_controller(nh);
    ROS_INFO("Moving Arms Now");
    arm_controller.moveToDefaultPose(armSide::RIGHT);
    ros::Duration(0.2).sleep();
    arm_controller.moveToDefaultPose(armSide::LEFT);
    ros::Duration(0.2).sleep();
}



void taskCommonUtils::moveToInitPose(ros::NodeHandle &nh)
{
    armTrajectory armTraj(nh);
    armTrajectory::armJointData l;
    l.side = LEFT;
    l.arm_pose = {-0.28f, -1.14f, 1.28f, -1.07f, 1.27f, 0.0f, 0.0f};
    l.time = 0.5;

    // Set the pose of the right arm to extend it to the front
    armTrajectory::armJointData r;
    r.side = RIGHT;
    r.arm_pose = {-0.28f, 1.14f, 1.28f, 1.07f, 1.27f, 0.0f, 0.0f};
    r.time = 0.5;

    // Combine the left and right arm movements
    std::vector<armTrajectory::armJointData> initPose;
    initPose.push_back(r);
    initPose.push_back(l);

    armTraj.moveArmJoints(initPose);
    ros::Duration(0.5).sleep();
}

void taskCommonUtils::fixHandFramePalmDown(ros::NodeHandle nh, armSide side, geometry_msgs::Pose &poseInWorldFrame){

    RobotStateInformer *current_state = RobotStateInformer::getRobotStateInformer(nh);
    geometry_msgs::Pose poseInPelvisFrame;
    current_state->transformPose(poseInWorldFrame, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);

    tf::Quaternion tfQuat;
    tf::quaternionMsgToTF(poseInPelvisFrame.orientation, tfQuat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);

    if (side == armSide::LEFT){
        pitch += M_PI_2;
        yaw   -= M_PI_2;
    }
    else {
        pitch += M_PI_2;
        roll  -= M_PI_2;
    }
    tf::Quaternion tfQuatOut = tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tfQuatOut, poseInPelvisFrame.orientation);
    current_state->transformPose(poseInPelvisFrame, poseInWorldFrame, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

}

void taskCommonUtils::fixHandFramePalmUp(ros::NodeHandle nh, armSide side, geometry_msgs::Pose &poseInWorldFrame)
{
    RobotStateInformer *current_state = RobotStateInformer::getRobotStateInformer(nh);
    geometry_msgs::Pose poseInPelvisFrame;
    current_state->transformPose(poseInWorldFrame, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);

    tf::Quaternion tfQuat;
    tf::quaternionMsgToTF(poseInPelvisFrame.orientation, tfQuat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);

    if (side == armSide::LEFT){
        roll  += M_PI / 6.0f;
        pitch -= 80.0*M_PI/180.0;
        yaw   -= M_PI * 4.0f/6.0f;
    }
    else {
        roll += M_PI / 6.0f;
        pitch -= 80.0*M_PI/180.0;
        roll  += M_PI / 3.0f;
    }
    tf::Quaternion tfQuatOut = tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tfQuatOut, poseInPelvisFrame.orientation);
    current_state->transformPose(poseInPelvisFrame, poseInWorldFrame, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
}
bool taskCommonUtils::slowGrip(ros::NodeHandle nh,armSide side, std::vector<double> initial, std::vector<double> final, int iterations, float executionTime)
{
    std::vector<double> diff,step;
    for (size_t i = 0; i < initial.size(); ++i) {
        diff.push_back((final[i]-initial[i])/iterations);
    }
    gripperControl gripper_controller(nh);
    for (size_t i = 1; i <= initial.size(); ++i) {
        step.clear();
        step.push_back(initial[0]+i*diff[0]);
        step.push_back(initial[1]+i*diff[1]);
        step.push_back(initial[2]+i*diff[2]);
        step.push_back(initial[3]+i*diff[3]);
        step.push_back(initial[4]+i*diff[4]);

        gripper_controller.controlGripper(side,step);
        ros::Duration(executionTime/iterations).sleep();
    }
    return true;

}
