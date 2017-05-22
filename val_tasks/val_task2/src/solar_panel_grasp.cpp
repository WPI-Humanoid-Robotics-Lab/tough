#include <val_task2/solar_panel_grasp.h>
#include <stdlib.h>
#include <stdio.h>
#include "val_task_common/val_task_common_utils.h"

solar_panel_handle_grabber::solar_panel_handle_grabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftMiddleFingerGroup", VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner("rightMiddleFingerGroup", VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
}

solar_panel_handle_grabber::~solar_panel_handle_grabber()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}


bool solar_panel_handle_grabber::grasp_handles(armSide side, const geometry_msgs::Pose &goal, float executionTime)
{

    const std::vector<float>* seed;
    std::string endEffectorFrame;
    float palmToFingerOffset;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        endEffectorFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.07;
    }
    else {
        seed = &rightShoulderSeed_;
        endEffectorFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.07;
    }


    ROS_INFO("opening grippers");
    gripper_.controlGripper(side, GRIPPER_STATE::OPEN_THUMB_IN_APPROACH);

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    std::vector< std::vector<float> > armData;
    armData.push_back(*seed);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();


    //move arm to given point with known orientation and higher z
    geometry_msgs::Pose finalGoal, intermGoal;

    current_state_->transformPose(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    intermGoal.position.z += 0.1;

    //transform that point back to world frame
    current_state_->transformPose(intermGoal, intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
    taskCommonUtils::fixHandFramePose(nh_, side, intermGoal);

    ROS_INFO("Moving at an intermidate point before goal");
    ROS_INFO_STREAM("Intermidiate goal"<<intermGoal);
    armTraj_.moveArmInTaskSpace(side, intermGoal, executionTime*2);
    ros::Duration(executionTime*2).sleep();

    //move arm to final position with known orientation

    current_state_->transformPose(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, endEffectorFrame);
    current_state_->transformPose(intermGoal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, endEffectorFrame);

    intermGoal.position.y += palmToFingerOffset;
//    intermGoal.position.z -= 0.02; //finger to center of palm in Z-axis of hand frame
    finalGoal.position.y  += palmToFingerOffset; // this is to compensate for the distance between palm frame and center of palm
//    finalGoal.position.z  -= 0.02; //finger to center of palm in Z-axis of hand frame


    //transform that point back to world frame
    current_state_->transformPose(finalGoal, finalGoal, endEffectorFrame, VAL_COMMON_NAMES::WORLD_TF);
    current_state_->transformPose(intermGoal, intermGoal, endEffectorFrame, VAL_COMMON_NAMES::WORLD_TF);
    taskCommonUtils::fixHandFramePose(nh_, side, finalGoal);

    ROS_INFO("Moving towards goal");
    ROS_INFO_STREAM("Final goal"<<finalGoal);
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(intermGoal);
    waypoints.push_back(finalGoal);
    ROS_INFO_STREAM(finalGoal);
    moveit_msgs::RobotTrajectory traj;
    if(side == armSide::LEFT)
    {
        left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }
    else
    {
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }
    ROS_INFO("Calculated Traj");
    wholebody_controller_->compileMsg(side, traj.joint_trajectory);

    ros::Duration(executionTime).sleep();

    geometry_msgs::Pose finalFramePose;
    finalFramePose.position.y += palmToFingerOffset;
    current_state_->transformPose(finalFramePose, finalFramePose, endEffectorFrame);

    float x_diff = (finalGoal.position.x - finalFramePose.position.x);
    float y_diff = (finalGoal.position.y - finalFramePose.position.y);
    float z_diff = (finalGoal.position.z - finalFramePose.position.z);

    if (x_diff*x_diff + y_diff*y_diff + z_diff*z_diff > 0.1){
        return false;
    }


    ROS_INFO("Closing grippers");
    gripper_.closeGripper(side);
    ros::Duration(0.3).sleep();
    return true;
}
