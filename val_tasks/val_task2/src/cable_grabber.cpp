#include "val_task2/cable_grabber.h"

cableGrabber::cableGrabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    /* Top Grip */

    leftHandOrientation_.quaternion.x = 0.604;
    leftHandOrientation_.quaternion.y = 0.434;
    leftHandOrientation_.quaternion.z = -0.583;
    leftHandOrientation_.quaternion.w = 0.326;

    //    /* Side Grip */
    //    leftHandOrientation_.quaternion.x = 0.155;
    //    leftHandOrientation_.quaternion.y = -0.061;
    //    leftHandOrientation_.quaternion.z = -0.696;
    //    leftHandOrientation_.quaternion.w = 0.699;

    //    /* Side Grip */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.094;
    //    rightHandOrientation_.quaternion.y = -0.027;
    //    rightHandOrientation_.quaternion.z = 0.973;
    //    rightHandOrientation_.quaternion.w = -0.209;

    /* Top Grip Flat Hand */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.576;
    //    rightHandOrientation_.quaternion.y = 0.397;
    //    rightHandOrientation_.quaternion.z = 0.632;
    //    rightHandOrientation_.quaternion.w = 0.332;

    /* Top Grip Slightly Bent Hand */
    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientation_.quaternion.x = 0.640;
    rightHandOrientation_.quaternion.y = -0.380;
    rightHandOrientation_.quaternion.z = -0.614;
    rightHandOrientation_.quaternion.w = -0.261;

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftPalm", VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner("rightPalm", VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
}

cableGrabber::~cableGrabber()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}

void cableGrabber::grasp_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{

    const std::vector<float>* seed;
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        finalOrientationStamped = &leftHandOrientation_;
    }
    else {
        seed = &rightShoulderSeed_;
        finalOrientationStamped = &rightHandOrientation_;
    }

    // opening gripper
    ROS_INFO("opening grippers");
    gripper_.openGripper(side);

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    std::vector< std::vector<float> > armData;
    armData.push_back(*seed);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();

    geometry_msgs::Pose finalGoal;
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;
    current_state_->transformQuaternion(temp, temp);

    ROS_INFO("Moving towards goal with whole body controller");
    finalGoal.position = goal;
    finalGoal.position.z+=0.05;
    finalGoal.orientation = temp.quaternion;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(finalGoal);

    moveit_msgs::RobotTrajectory traj;
    right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    wholebody_controller_->compileMsg(side, traj.joint_trajectory);

    gripper_.controlGripper(side,GRIPPER_STATE::OPEN_THUMB_IN_APPROACH);
    ros::Duration(0.3).sleep();

    std::vector<double> grip1 ={1.3999, 0.3, 0.5, 0.0, 0.0};
    std::vector<double> grip2 ={1.3999, 0.5, 0.5, 0.0, 0.0};
    std::vector<double> grip3 ={1.3999, 0.7, 0.6, 0.0, 0.0};
    std::vector<double> grip4 ={1.3999, 0.9, 0.7, 0.0, 0.0};

    gripper_.controlGripper(side,grip1);
    ros::Duration(0.3).sleep();
    gripper_.controlGripper(side,grip2);
    ros::Duration(0.3).sleep();
    gripper_.controlGripper(side,grip3);
    ros::Duration(0.3).sleep();
    gripper_.controlGripper(side,grip4);
    ros::Duration(0.3).sleep();

    // nudging arm forward
    ros::Duration(0.3).sleep();
    armTraj_.nudgeArmLocal(side,direction::FRONT);

    // closing grippers
    ros::Duration(executionTime).sleep();
    ROS_INFO("Closing grippers");
    gripper_.closeGripper(side);
    ros::Duration(0.3).sleep();
}
