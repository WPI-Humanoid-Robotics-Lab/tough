#include <val_task2/button_press.h>

button_press::button_press(ros::NodeHandle& nh):nh_(nh), armTraj_(nh), gripper_(nh), bd_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);


    /* Top Grip */
    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientation_.quaternion.x = 0.604;
    leftHandOrientation_.quaternion.y = 0.434;
    leftHandOrientation_.quaternion.z = -0.583;
    leftHandOrientation_.quaternion.w = 0.326;


    /* Top Grip Flat Hand */
    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientation_.quaternion.x = -0.549;
    rightHandOrientation_.quaternion.y = 0.591;
    rightHandOrientation_.quaternion.z = 0.560;
    rightHandOrientation_.quaternion.w = 0.188;


    //    /* Side Grip */
    //    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    leftHandOrientation_.quaternion.x = -0.050;
    //    leftHandOrientation_.quaternion.y = -0.027;
    //    leftHandOrientation_.quaternion.z = -0.678;
    //    leftHandOrientation_.quaternion.w = 0.732;

    //    /* Side Grip */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.023;
    //    rightHandOrientation_.quaternion.y = 0.007;
    //    rightHandOrientation_.quaternion.z = 0.723;
    //    rightHandOrientation_.quaternion.w = 0.691;

    // Initializing planners
    left_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
    chest_controller_ = new chestTrajectory(nh_);

}

button_press::~button_press()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
    delete chest_controller_;
}

bool button_press::pressButton(const armSide side, geometry_msgs::Point &goal, float executionTime)
{
    // setting initial values
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    const std::vector<float>* armSeed;
    std::string palmFrame;
    float palmToFingerOffset;
    if(side == armSide::LEFT){
        armSeed = &leftShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.12;  // offset between middle finger and palm
        finalOrientationStamped = &leftHandOrientation_;
    }
    else {
        armSeed = &rightShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.12; // offset between middle finger and palm
        finalOrientationStamped = &rightHandOrientation_;
    }
    std::vector< std::vector<float> > armData;
    armData.push_back({0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    // moving both arms to zero position
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    armTraj_.moveArmJoints(RIGHT, armData, executionTime);
    ros::Duration(executionTime*2).sleep();


    // getting the orientation
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;
    current_state_->transformQuaternion(temp, temp);

    ROS_INFO("closing grippers");
    gripper_.controlGripper(side,GRIPPER_STATE::OPEN_THUMB_IN);
    ros::Duration(executionTime).sleep();

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    armData.clear();
    armData.push_back(*armSeed);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();


    //move arm to given point with known orientation and higher z
    geometry_msgs::Point finalGoal, intermGoal;

    current_state_->transformPoint(goal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    intermGoal.z += 0.2;

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

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose final;

    final.position=intermGoal;
    final.orientation= temp.quaternion;

    waypoints.push_back(final);

    final.position=finalGoal;
    final.position.z+=0.08; // to press the button
    final.orientation= temp.quaternion;

    waypoints.push_back(final);

    final.position=finalGoal;
    final.position.z+=0.25; // to release the button
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

    ROS_INFO("Moving chest to zero position");
    chest_controller_->controlChest(0,0,0);

    armData.clear();
    armData.push_back(*armSeed);
    ROS_INFO("Moving arms to intermediate position");
    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();
    return true;
}

void button_press::getButtonPosition( geometry_msgs::Point &goal)
{
    bd_.findButtons(goal);

}

