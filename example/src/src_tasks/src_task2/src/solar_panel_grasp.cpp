#include <src_task2/solar_panel_grasp.h>
#include <stdlib.h>
#include <stdio.h>
#include "src_task_common/val_task_common_utils.h"
#include "tough_control_common/tough_control_common.h"

solar_panel_handle_grabber::solar_panel_handle_grabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    // cartesian planners for the arm
    left_arm_planner_ = new CartesianPlanner("leftMiddleFingerGroup", VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new CartesianPlanner("rightMiddleFingerGroup", VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new WholebodyControlInterface(nh_);
}

solar_panel_handle_grabber::~solar_panel_handle_grabber()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}


bool solar_panel_handle_grabber::grasp_handles(RobotSide side, const geometry_msgs::Pose &goal, bool isRotationRequired, float executionTime)
{

    const std::vector<float>* seed;
    std::string endEffectorFrame;
    float palmToFingerOffset;

    if(side == RobotSide::LEFT){
        seed = &leftShoulderSeed_;
        endEffectorFrame = rd_->getLeftEEFrame();
        palmToFingerOffset = 0.07;
    }
    else {
        seed = &rightShoulderSeed_;
        endEffectorFrame = rd_->getRightEEFrame();
        palmToFingerOffset = -0.07;
    }
    ToughControlCommon control_util(nh_);

    ROS_INFO("solar_panel_handle_grabber::grasp_handles : opening grippers");
    if (side == RIGHT)
    {
        if(isRotationRequired){
            gripper_.controlGripper(RIGHT, {1.39,0.3,0.1,0.1,0.1});
        }
        else{
            gripper_.openGripper(RIGHT);
        }
    }
    else {
        if (isRotationRequired){
            gripper_.controlGripper(LEFT, {1.39,-0.3,-0.1,-0.1,-0.1});
        }
        else {
            gripper_.openGripper(LEFT);
        }
    }
    ros::Duration(0.2).sleep();

    //move arm to given point with known orientation and higher z
    geometry_msgs::Pose finalGoal, intermGoal;

    current_state_->transformPose(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
    float yaw = tf::getYaw(intermGoal.orientation);
    if (isRotationRequired){
        intermGoal.position.x -= 0.2*cos(yaw);
        intermGoal.position.y -= 0.2*sin(yaw);
    }
    else{
        intermGoal.position.x += 0.05*cos(yaw);
        intermGoal.position.y += 0.05*sin(yaw);
        intermGoal.position.z += 0.1;
    }

    //transform that point back to world frame
    current_state_->transformPose(intermGoal, intermGoal, rd_->getPelvisFrame(), VAL_COMMON_NAMES::WORLD_TF);

    if (isRotationRequired){
        taskCommonUtils::fixHandFramePalmUp(nh_, side, intermGoal);
    }
    else{
        taskCommonUtils::fixHandFramePalmDown(nh_,side, intermGoal);
    }

    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Moving at an intermidate point before goal");
    ROS_INFO("Intermidiate goal x:%f y:%f z:%f quat x:%f y:%f z:%f w:%f",intermGoal.position.x, intermGoal.position.y, intermGoal.position.z,
             intermGoal.orientation.x, intermGoal.orientation.y, intermGoal.orientation.z, intermGoal.orientation.w);
    armTraj_.moveArmInTaskSpace(side, intermGoal, executionTime);
    ros::Duration(executionTime*2).sleep();
    control_util.stopAllTrajectories();

    //move arm to final position with known orientation

    current_state_->transformPose(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
    if (isRotationRequired){
        finalGoal.position.x += 0.1*cos(yaw);
        finalGoal.position.y += 0.1*sin(yaw);
    }
    else{
        finalGoal.position.x += 0.05*cos(yaw);
        finalGoal.position.y += 0.05*sin(yaw);
        finalGoal.position.z -= 0.05;
    }

    //transform that point back to world frame
    current_state_->transformPose(finalGoal, finalGoal, rd_->getPelvisFrame(), VAL_COMMON_NAMES::WORLD_TF);

    if (isRotationRequired){
        taskCommonUtils::fixHandFramePalmUp(nh_, side, finalGoal);
        intermGoal.position.z -= 0.06;
        finalGoal.position.z  -= 0.06;
    }
    else {
        taskCommonUtils::fixHandFramePalmDown(nh_, side, finalGoal);
    }
    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Moving towards goal");
    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Final goal  x:%f y:%f z:%f quat x:%f y:%f z:%f w:%f",finalGoal.position.x, finalGoal.position.y,
             finalGoal.position.z, finalGoal.orientation.x, finalGoal.orientation.y, finalGoal.orientation.z, finalGoal.orientation.w);
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(intermGoal);
    waypoints.push_back(finalGoal);

    if (isRotationRequired){
        finalGoal.position.z  -= 0.01;
        waypoints.push_back(finalGoal);

        finalGoal.position.z  += 0.03;
        waypoints.push_back(finalGoal);
    }

    moveit_msgs::RobotTrajectory traj;
    if(side == RobotSide::LEFT)
    {
        if (left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false) < 0.98){
            ROS_INFO("solar_panel_handle_grabber::grasp_handles : Trajectory is not planned 100% - retrying");
            return false;
        }
    }
    else
    {
        if (right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false)< 0.98){
            ROS_INFO("solar_panel_handle_grabber::grasp_handles : Trajectory is not planned 100% - retrying");
            return false;
        }
    }
    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Calculated Traj");
    wholebody_controller_->executeTrajectory(side, traj.joint_trajectory);
    ros::Duration(executionTime*2).sleep();

    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Closing grippers");
    if (isRotationRequired){
        gripper_.controlGripper(side,GRIPPER_STATE::CUP);
    }
    else {
        gripper_.controlGripper(side,GRIPPER_STATE::TIGHT_HOLD);
    }
    ros::Duration(0.3).sleep();

    finalGoal.position.z  += 0.2;
    waypoints.clear();
    waypoints.push_back(finalGoal);

    moveit_msgs::RobotTrajectory traj2;
    if(side == RobotSide::LEFT)
    {
        if (left_arm_planner_->getTrajFromCartPoints(waypoints, traj2, false) < 0.98){
            ROS_INFO("solar_panel_handle_grabber::grasp_handles : Trajectory is not planned 100% - retrying");
            return false;
        }
    }
    else
    {
        if (right_arm_planner_->getTrajFromCartPoints(waypoints, traj2, false)< 0.98){
            ROS_INFO("solar_panel_handle_grabber::grasp_handles : Trajectory is not planned 100% - retrying");
            return false;
        }
    }

    wholebody_controller_->executeTrajectory(side, traj2.joint_trajectory);
    ros::Duration(executionTime*2).sleep();

    geometry_msgs::Pose finalFramePose;

    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Fecthing position of %s", endEffectorFrame.c_str());
    current_state_->getCurrentPose(endEffectorFrame,finalFramePose, VAL_COMMON_NAMES::WORLD_TF);

    float x_diff = (finalGoal.position.x - finalFramePose.position.x);
    float y_diff = (finalGoal.position.y - finalFramePose.position.y);
    float z_diff = (finalGoal.position.z - finalFramePose.position.z);

    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Expected Goal Pose : %f %f %f", finalGoal.position.x, finalGoal.position.y, finalGoal.position.z);
    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Actual Finger Pose : %f %f %f", finalFramePose.position.x, finalFramePose.position.y, finalFramePose.position.z);
    ROS_INFO("solar_panel_handle_grabber::grasp_handles : Distance between final pose and goal is %f", sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff));

    //    if (sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff) > 0.1){
    //        return false;
    //    }

    return true;
}
