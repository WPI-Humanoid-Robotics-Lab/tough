#include <val_task3/leak_detector_grabber.h>
#include <stdlib.h>
#include <stdio.h>


leakDetectorGrabber::leakDetectorGrabber(ros::NodeHandle nh):nh_(nh),
              armTraj_(nh_), gripper_(nh_), wholebody_controller_(nh_)
              {
    right_arm_planner_ = new cartesianPlanner("rightPalm");
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
//    right_arm_planner_ = cartesianPlanner("rightpalm", VAL_COMMON_NAMES::WORLD_TF);
//    left_arm_planner_ = cartesianPlanner("leftpalm", VAL_COMMON_NAMES::WORLD_TF);
}

leakDetectorGrabber::~leakDetectorGrabber(){

}

void leakDetectorGrabber::graspDetector(const geometry_msgs::Pose &goal, float executionTime){

    //geometry_msgs::Pose intermediatePose;
    geometry_msgs::Pose finalGoal, intermGoal;
    //geometry_msgs::Point finalPoint, intermPoint;

    current_state_->transformPose(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    intermGoal.position.z += 0.1;

    //transform that point back to world frame
    current_state_->transformPose(intermGoal, intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving at an intermidate point before goal");
    armTraj_.moveArmInTaskSpace(RIGHT, intermGoal, executionTime*2);
    ros::Duration(executionTime*2).sleep();

    //move arm to final position with known orientation

//    current_state_->transformPose(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
//    finalGoal.position.x -= 0.04; // this is to compensate for the distance between palm frame and center of palm

//    //transform that point back to world frame
//    current_state_->transformPose(finalGoal, finalGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

//    ROS_INFO("Moving towards goal");

//    std::vector<geometry_msgs::Pose> waypoints;

//    waypoints.push_back(finalGoal);
//    ROS_INFO_STREAM(finalGoal);
//    moveit_msgs::RobotTrajectory traj;
//    right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);

//    ROS_INFO("Calculated Traj");
//    wholebody_controller_->compileMsg(side, traj.joint_trajectory);

//    ros::Duration(executionTime).sleep();
//    ROS_INFO("Closing grippers");
//    gripper_.closeGripper(side);
//    ros::Duration(0.3).sleep();
}

