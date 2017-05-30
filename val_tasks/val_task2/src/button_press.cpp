#include <val_task2/button_press.h>

ButtonPress::ButtonPress(ros::NodeHandle& nh):nh_(nh), armTraj_(nh), gripper_(nh), bd_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);


    /* Top Grip */
    //    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    leftHandOrientation_.quaternion.x = 0.604;
    //    leftHandOrientation_.quaternion.y = 0.434;
    //    leftHandOrientation_.quaternion.z = -0.583;
    //    leftHandOrientation_.quaternion.w = 0.326;

    //Most recent//
    //    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    leftHandOrientation_.quaternion.x = 0.533;
    //    leftHandOrientation_.quaternion.y = 0.373;
    //    leftHandOrientation_.quaternion.z = -0.430;
    //    leftHandOrientation_.quaternion.w = 0.627;

    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientation_.quaternion.x = 0.492;
    leftHandOrientation_.quaternion.y = 0.504;
    leftHandOrientation_.quaternion.z = -0.494;
    leftHandOrientation_.quaternion.w = 0.509;

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
    left_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_PALM_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
    chest_controller_ = new chestTrajectory(nh_);

}

ButtonPress::~ButtonPress()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
    delete chest_controller_;
}

bool ButtonPress::pressButton(const armSide side, geometry_msgs::Point &goal, float executionTime)
{
    // setting initial values
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    const std::vector<float>* armSeed, *initialSeed;
    std::string palmFrame;
    float xFingerOffset,yFingerOffset,zFingerOffset;
    geometry_msgs::Pose rightOffset,leftOffset;
    current_state_->getCurrentPose("/rightMiddleFingerPitch1Link",rightOffset,"/rightThumbRollLink");
    current_state_->getCurrentPose("/leftMiddleFingerPitch1Link",leftOffset,"/leftThumbRollLink");
    if(side == armSide::LEFT){
        armSeed = &leftShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::LEFT_PALM_GROUP;
        xFingerOffset = leftOffset.position.x;
        yFingerOffset = leftOffset.position.y;// minor offsets to hit centeto hit center of buttonr of button
        zFingerOffset = leftOffset.position.z; // minor offsets to hit center of buttonto hit center of button

        finalOrientationStamped = &leftHandOrientation_;
    }
    else {
        armSeed = &rightShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        xFingerOffset = rightOffset.position.x;
        yFingerOffset = rightOffset.position.y;
        zFingerOffset = rightOffset.position.z;
        finalOrientationStamped = &rightHandOrientation_;
    }
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

    ROS_INFO("setting gripper position thumb-in");
    gripper_.controlGripper(side,GRIPPER_STATE::OPEN_THUMB_IN);
    ros::Duration(executionTime).sleep();

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    armData.clear();
    armData.push_back(*armSeed);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime).sleep();

    //move arm to given point with known orientation and higher z
    geometry_msgs::Point finalGoal;

    current_state_->transformPoint(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF,palmFrame);
    finalGoal.x+= xFingerOffset;
    finalGoal.y+= yFingerOffset;
    finalGoal.z+= zFingerOffset;


    //transform that point back to world frame
    current_state_->transformPoint(finalGoal, finalGoal, palmFrame, VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving towards goal");

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose final;

    final.position=finalGoal;
    final.position.z+=0.20; // position above the button
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
    armData.push_back(leftShoulderSeedInitial_);
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();
    armData.clear();
    armData.push_back(rightShoulderSeedInitial_);
    armTraj_.moveArmJoints(RIGHT, armData, executionTime);
    ros::Duration(executionTime).sleep();
    return true;
}

void ButtonPress::getButtonPosition( geometry_msgs::Point &goal)
{
    bd_.findButtons(goal);

}

