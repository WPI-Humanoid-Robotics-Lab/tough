#include <val_task3/leak_detector_grabber.h>
#include <stdlib.h>
#include <stdio.h>


leakDetectorGrabber::leakDetectorGrabber(ros::NodeHandle nh):nh_(nh),
              armTraj_(nh_), gripper_(nh_), wholebody_controller_(nh_)
              {
    right_arm_planner_ = new cartesianPlanner("rightPalm");
    left_arm_planner_ = new cartesianPlanner("leftPalm");
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

leakDetectorGrabber::~leakDetectorGrabber(){

}


void leakDetectorGrabber::graspDetector(armSide side, geometry_msgs::Pose &goal, float executionTime){

    std::string palmFrame;
    float palmToFingerOffset;

    if(side == armSide::LEFT){
        palmFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.04;
    }
    else {
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.04;
    }

    ROS_INFO("opening grippers");
    gripper_.openGripper(side);

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

    current_state_->transformPose(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);
    current_state_->transformPose(intermGoal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);

    intermGoal.position.y += palmToFingerOffset;
    intermGoal.position.z -= 0.02; //finger to center of palm in Z-axis of hand frame
    finalGoal.position.y  += palmToFingerOffset; // this is to compensate for the distance between palm frame and center of palm
    finalGoal.position.z  -= 0.02; //finger to center of palm in Z-axis of hand frame

    //transform that point back to world frame
        current_state_->transformPose(finalGoal, finalGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);
        current_state_->transformPose(intermGoal, intermGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);
        taskCommonUtils::fixHandFramePose(nh_, side, finalGoal);

        ROS_INFO("Moving towards goal");
        ROS_INFO_STREAM("Final goal"<<finalGoal);
        std::vector<geometry_msgs::Pose> waypoints;

        waypoints.push_back(intermGoal);
        waypoints.push_back(finalGoal);

        ROS_INFO_STREAM(finalGoal);
        moveit_msgs::RobotTrajectory traj;

        if(side == armSide::LEFT){

            left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
        }
        else{

            right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
        }

        ROS_INFO("Calculated Traj");
        wholebody_controller_.compileMsg(side, traj.joint_trajectory);

        ros::Duration(executionTime).sleep();
        ROS_INFO("Closing grippers");
        gripper_.closeGripper(side);
        ros::Duration(0.3).sleep();

//    geometry_msgs::Pose finalGoal, intermGoal;
//    std::vector<geometry_msgs::Pose> waypoints;
//    moveit_msgs::RobotTrajectory traj;

//    taskCommonUtils::fixHandFramePose(nh,side,goal);

//    current_state_->transformPose(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
//    intermGoal.position.z += 0.1;
//    current_state_->transformPose(intermGoal, intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

//    ROS_INFO("Moving at an intermidate point before goal");
//    armTraj_.moveArmInTaskSpace(side, intermGoal, executionTime*2);
//    ros::Duration(executionTime*2).sleep();

//    current_state_->transformPose(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
//   // finalGoal.position.x -= 0.04;
//   // finalGoal.position.y += 0.01;
//    finalGoal.position.z += 0.002;
//    current_state_->transformPose(finalGoal, finalGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

//    ROS_INFO("Moving towards goal");

//    waypoints.push_back(finalGoal);
//    ROS_INFO_STREAM(finalGoal);
//    right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);

//    ROS_INFO("Calculated Traj");
//    wholebody_controller_.compileMsg(side, traj.joint_trajectory);

//    ros::Duration(executionTime).sleep();
//    ROS_INFO("Closing grippers");
//    ros::Duration(0.3).sleep();
//    gripper_.closeGripper(side);
//    ros::Duration(0.3).sleep();
}

