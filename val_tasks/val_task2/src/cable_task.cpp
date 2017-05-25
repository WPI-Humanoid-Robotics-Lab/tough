#include "val_task2/cable_task.h"


cableTask::cableTask(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    leftHandOrientationTop_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    /* Top Grip */

    leftHandOrientationTop_.quaternion.x = 0.604;
    leftHandOrientationTop_.quaternion.y = 0.434;
    leftHandOrientationTop_.quaternion.z = -0.583;
    leftHandOrientationTop_.quaternion.w = 0.326;

    /* Top Grip Flat Hand modified*/
    rightHandOrientationTop_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientationTop_.quaternion.x = -0.459;
    rightHandOrientationTop_.quaternion.y = 0.550;
    rightHandOrientationTop_.quaternion.z = 0.602;
    rightHandOrientationTop_.quaternion.w = 0.353;

    /* Side Grip */
    leftHandOrientationSide_.quaternion.x = 0.155;
    leftHandOrientationSide_.quaternion.y = -0.061;
    leftHandOrientationSide_.quaternion.z = -0.696;
    leftHandOrientationSide_.quaternion.w = 0.699;

    /* Side Grip */
    rightHandOrientationSide_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientationSide_.quaternion.x = -0.094;
    rightHandOrientationSide_.quaternion.y = -0.027;
    rightHandOrientationSide_.quaternion.z = 0.973;
    rightHandOrientationSide_.quaternion.w = -0.209;

    /* Top Grip Flat Hand */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.576;
    //    rightHandOrientation_.quaternion.y = 0.397;
    //    rightHandOrientation_.quaternion.z = 0.632;
    //    rightHandOrientation_.quaternion.w = 0.332;

    /* Top Grip Slightly Bent Hand */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = 0.640;
    //    rightHandOrientation_.quaternion.y = -0.380;
    //    rightHandOrientation_.quaternion.z = -0.614;
    //    rightHandOrientation_.quaternion.w = -0.261;

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_PALM_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
    chest_controller_ = new chestTrajectory(nh_);
}

cableTask::~cableTask()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
    delete chest_controller_;
}

bool cableTask::grasp_cable(const geometry_msgs::Point &goal, float executionTime)
{
    /* set orientation desired ( need to get this from perception ) @Todo change later!
     * set seed point. one standard initial point to avoid collision and the other closer to the cable
     * set grasping pose. approach from the thumb in the bottom. close all fingers to grasp
     * set last seed position based on how to approach the socket
     */

    // Setting initial positions for both arms
    ROS_INFO("grasp_cable: Initial Pose");
    std::vector< std::vector<float> > armData;
    armData.push_back(leftShoulderSeedInitial_);
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    armData.clear();
    armData.push_back(rightShoulderSeedInitial_);
    armData.push_back(rightShoulderSeed_);
    armTraj_.moveArmJoints(RIGHT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    // setting orientation of final pose
    geometry_msgs::QuaternionStamped temp  =rightHandOrientationTop_;
    current_state_->transformQuaternion(temp,temp);

    geometry_msgs::Pose rightOffset;
    current_state_->getCurrentPose("/rightPalm",rightOffset,"/rightThumbRollLink");
    geometry_msgs::Point intermGoal;
    current_state_->transformPoint(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::R_PALM_TF);
    intermGoal.x+=rightOffset.position.x;
    intermGoal.y+=rightOffset.position.y;
    intermGoal.z+=rightOffset.position.z;
    current_state_->transformPoint(intermGoal,intermGoal, VAL_COMMON_NAMES::R_PALM_TF, VAL_COMMON_NAMES::WORLD_TF);

    float offset=0.03;
    geometry_msgs::Pose final;
    final.position=intermGoal;
    final.position.z+=offset;
    final.orientation=temp.quaternion;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(final);
    moveit_msgs::RobotTrajectory traj;

    // Sending waypoints to the planner
    right_arm_planner_->getTrajFromCartPoints(waypoints,traj,false);

    //    // Opening grippers
    //    ROS_INFO("grasp_cable: Setting gripper position to open thumb");
    //    std::vector<double> gripper1;
    //    gripper1={1.35,0.5,0.0,0.0,0.0};
    //    gripper_.controlGripper(RIGHT,gripper1);
    //    ros::Duration(executionTime/2).sleep();

    //    // Planning whole body motion
    //    wholebody_controller_->compileMsg(RIGHT,traj);
    //    ros::Duration(executionTime*2).sleep();

    //    // Grasping the cable
    //    gripper_.closeGripper(RIGHT);

}

bool cableTask::insert_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{
    /*
     * calculate difference in (x,y) position of middlefinger and socket point
     * subscribe to the topic /srcsim/finals/task to check when the cable touches
     * lower the cable( to a defined depth) till you see the touch message
     * if in touch position stop moving and wait for completion. release the cable when completed
     * if none of the above happens, do that in a circular area of defined radius
    */
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    if(side == armSide::LEFT){
        finalOrientationStamped = &leftHandOrientationSide_;
    }
    else{
        finalOrientationStamped = &rightHandOrientationSide_;
    }

    ROS_INFO("Setting initial pose");
    std::vector< std::vector<float> > armData;
    armData.push_back(leftShoulderSeedInitial_);
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();
    armData.clear();
    armData.push_back(rightShoulderSeedInitial_);
    armTraj_.moveArmJoints(RIGHT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    // getting the orientation
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;
    current_state_->transformQuaternion(temp, temp);


    geometry_msgs::Pose finalPoint;
    std::vector<geometry_msgs::Pose> waypoints;


    finalPoint.position=goal;
    finalPoint.position.z+=0.3;
    finalPoint.orientation=temp.quaternion;
    waypoints.push_back(finalPoint);

    finalPoint.position=goal;
    finalPoint.position.z+=0.2;
    finalPoint.orientation=temp.quaternion;
    waypoints.push_back(finalPoint);


    moveit_msgs::RobotTrajectory traj;
    if(side == armSide::LEFT)
    {
        left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }
    else
    {
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }

    ROS_INFO("Moving to goal");
    wholebody_controller_->compileMsg(side, traj.joint_trajectory);
    ros::Duration(executionTime).sleep();

}
