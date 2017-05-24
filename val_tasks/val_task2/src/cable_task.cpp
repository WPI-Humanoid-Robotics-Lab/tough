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

    /* Top Grip Flat Hand */
    rightHandOrientationTop_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientationTop_.quaternion.x = -0.549;
    rightHandOrientationTop_.quaternion.y = 0.591;
    rightHandOrientationTop_.quaternion.z = 0.560;
    rightHandOrientationTop_.quaternion.w = 0.188;

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
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
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

bool cableTask::grasp_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{
    // setting initial values
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    const std::vector<float>* seed;
    const std::vector<float>* seedAfter;
    std::string palmFrame;
    float palmToFingerOffset;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        seedAfter=&leftShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.07;
        finalOrientationStamped = &leftHandOrientationTop_;
    }
    else {
        seed = &rightShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.07;
        finalOrientationStamped = &rightHandOrientationTop_;
        seedAfter=&rightAfterGraspShoulderSeed_;
    }

    // getting the orientation
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;
    current_state_->transformQuaternion(temp, temp);

    ROS_INFO("opening grippers");
    std::vector<double> gripper1,gripper2,gripper3;
    gripper1={1.2, 0.4, 0.3, 0.0 ,0.0 };
    gripper2={1.2, 0.6, 0.7, 0.0 ,0.0 };
    gripper3={1.2, 0.6, 0.7, 0.9 ,1.0 };
    gripper_.controlGripper(side,gripper1);
    ros::Duration(executionTime).sleep();

    //move shoulder roll outwards
    ROS_INFO("Initial Pose");
    std::vector< std::vector<float> > armData;

    armData.push_back(leftShoulderSeedInitial_);
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();
    armData.clear();
    armData.push_back(rightShoulderSeedInitial_);
    armTraj_.moveArmJoints(RIGHT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    armData.clear();
    armData.push_back(*seed);

    ROS_INFO("Moving closer to cable");
    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();


    //move arm to given point with known orientation and higher z
    geometry_msgs::Point finalGoal, intermGoal;

    current_state_->transformPoint(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    intermGoal.z += 0.1;

    //transform that point back to world frame
    current_state_->transformPoint(intermGoal, intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving at an intermidate point before goal");


    current_state_->transformPoint(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);
    current_state_->transformPoint(intermGoal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);

    intermGoal.y += palmToFingerOffset;
    finalGoal.y  += palmToFingerOffset; // this is to compensate for the distance between palm frame and center of palm

    //transform that point back to world frame
    current_state_->transformPoint(finalGoal, finalGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);
    current_state_->transformPoint(intermGoal, intermGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving towards goal");
    ROS_INFO_STREAM("Final goal"<<finalGoal);
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose final;

    final.position=intermGoal;
    final.orientation= temp.quaternion;

    waypoints.push_back(final);

    final.position=finalGoal;
    final.position.z+=0.01; // offset between cable and table
    final.orientation= temp.quaternion;

    waypoints.push_back(final);

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

    ROS_INFO("Grip Sequence 1");
    gripper_.controlGripper(side, gripper1);
    ros::Duration(0.1).sleep();
    ROS_INFO("Grip Sequence 2");
    gripper_.controlGripper(side, gripper2);
    ros::Duration(0.1).sleep();
    ROS_INFO("Grip Sequence 3");
    gripper_.controlGripper(side, gripper3);
    ros::Duration(0.1).sleep();

    ROS_INFO("Moving chest to zero position");
    chest_controller_->controlChest(0,0,0);

    armData.clear();
    armData.push_back(*seedAfter);
    ROS_INFO("Moving arms to intermediate position");
    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();
    return true;

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
