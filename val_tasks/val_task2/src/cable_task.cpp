#include <val_task2/cable_task.h>
#include <stdlib.h>
#include <stdio.h>
#include "val_task_common/val_task_common_utils.h"
#include "val_control_common/val_control_common.h"

CableTask::CableTask(ros::NodeHandle n, BUTTON_LOCATION button_side):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    // cartesian planners for the arm
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
    chest_controller_ =new chestTrajectory(nh_);
}

CableTask::~CableTask()
{
    delete right_arm_planner_;
    delete wholebody_controller_;
    delete chest_controller_;
}


bool CableTask::grasp_cable(const geometry_msgs::Pose &goal, float executionTime)
{

    std::string endEffectorFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
    float palmToFingerOffset = -0.07;
    armSide side = RIGHT;

    valControlCommon control_util(nh_);

    ROS_INFO("CableTask::grasp_cable : opening grippers");
    gripper_.controlGripper(side, GRIPPER_STATE::OPEN);

    //move shoulder roll outwards
    ROS_INFO("CableTask::grasp_cable : Setting initial positions");

    std::vector< std::vector<float> > armData;
    armData.push_back(leftShoulderSeedInitial_);
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(0.2).sleep();

    armData.clear();
        armData.push_back(rightShoulderSeedInitial_);
//    armData.push_back({0.1,0.1,0.1,0.1,0.1,0.1,0.1});
    armTraj_.moveArmJoints(RIGHT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    ROS_INFO("CableTask::grasp_cable : initial positions have been set");
    //    control_util.stopAllTrajectories();

    //move arm to given point with known orientation and higher z
    geometry_msgs::Pose finalGoal, intermGoal;


    current_state_->transformPose(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    intermGoal.position.z += 0.1;

    //transform that point back to world frame
    current_state_->transformPose(intermGoal, intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);


    taskCommonUtils::fixHandFramePalmDown(nh_, side, intermGoal);

    ROS_INFO("CableTask::grasp_cable : Moving at an intermidate point before goal");
    ROS_INFO("CableTask::grasp_cable : Intermidiate goal x:%f y:%f z:%f quat x:%f y:%f z:%f w:%f",intermGoal.position.x, intermGoal.position.y, intermGoal.position.z,
             intermGoal.orientation.x, intermGoal.orientation.y, intermGoal.orientation.z, intermGoal.orientation.w);

    armTraj_.moveArmInTaskSpace(side, intermGoal, executionTime);
    ros::Duration(executionTime*2).sleep();
    control_util.stopAllTrajectories();

    //move arm to final position with known orientation

    //        current_state_->transformPose(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    //        finalGoal.position.z -= 0.05;

    //transform that point back to world frame
    //        current_state_->transformPose(finalGoal, finalGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
    finalGoal = goal;
    finalGoal.position.z-=0.01;
    taskCommonUtils::fixHandFramePalmDown(nh_, side, finalGoal);

    ROS_INFO("CableTask::grasp_cable: Moving towards goal");
    ROS_INFO("CableTask::grasp_cable: Final goal  x:%f y:%f z:%f quat x:%f y:%f z:%f w:%f",finalGoal.position.x, finalGoal.position.y,
             finalGoal.position.z, finalGoal.orientation.x, finalGoal.orientation.y, finalGoal.orientation.z, finalGoal.orientation.w);
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(intermGoal);
    waypoints.push_back(finalGoal);

    moveit_msgs::RobotTrajectory traj;


    if (right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false)< 0.98){
        ROS_INFO("CableTask::grasp_cable: Trajectory is not planned 100% - retrying");
        return false;
    }

//    ROS_INFO("CableTask::grasp_cable: Calculated Traj");
//    wholebody_controller_->compileMsg(side, traj.joint_trajectory);
//    ros::Duration(executionTime*2).sleep();

//    geometry_msgs::Pose finalFramePose;
//    ROS_INFO("CableTask::grasp_cable: Fecthing position of %s", endEffectorFrame.c_str());
//    current_state_->getCurrentPose(endEffectorFrame,finalFramePose, VAL_COMMON_NAMES::WORLD_TF);

//    float x_diff = (finalGoal.position.x - finalFramePose.position.x);
//    float y_diff = (finalGoal.position.y - finalFramePose.position.y);
//    float z_diff = (finalGoal.position.z - finalFramePose.position.z);

//    ROS_INFO("CableTask::grasp_cable: Expected Goal Pose : %f %f %f", finalGoal.position.x, finalGoal.position.y, finalGoal.position.z);
//    ROS_INFO("CableTask::grasp_cable: Actual Finger Pose : %f %f %f", finalFramePose.position.x, finalFramePose.position.y, finalFramePose.position.z);
//    ROS_INFO("CableTask::grasp_cable: Distance between final pose and goal is %f", sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff));

//    //    if (sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff) > 0.1){
//    //        return false;
//    //    }



//    gripper_.closeGripper(side);
//    ros::Duration(0.3).sleep();

//    // Setting arm position to dock
//    ROS_INFO("grasp_cable: Setting arm position to dock cable");
//    armData.clear();
//    armData.push_back(rightAfterGraspShoulderSeed_);
//    armTraj_.moveArmJoints(RIGHT,armData,executionTime);
//    ros::Duration(0.3).sleep();

//    chest_controller_->controlChest(0,0,0);
//    ros::Duration(0.3).sleep();
    return true;
}
