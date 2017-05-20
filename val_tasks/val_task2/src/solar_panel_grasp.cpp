#include <val_task2/solar_panel_grasp.h>
#include <stdlib.h>
#include <stdio.h>


solar_panel_handle_grabber::solar_panel_handle_grabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);


    //    left hand - panel angled		-0.587, -0.327,  0.658, -0.339
    //    left hand - panel perpendicular		 0.697,  0.232, -0.664,  0.142

    //    right hand - panel angled		-0.631,  0.339,  0.680,  0.159
    //    right hand - panel perpendicular	 0.781, -0.155, -0.605, -0.020
    leftHandOrientationAngled_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientationAngled_.quaternion.x = -0.587;
    leftHandOrientationAngled_.quaternion.y = -0.327;
    leftHandOrientationAngled_.quaternion.z =  0.658;
    leftHandOrientationAngled_.quaternion.w = -0.339;

    leftHandOrientationPerpen_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientationPerpen_.quaternion.x =  0.697;
    leftHandOrientationPerpen_.quaternion.y =  0.232;
    leftHandOrientationPerpen_.quaternion.z = -0.664;
    leftHandOrientationPerpen_.quaternion.w =  0.142;

    /* Top Grip */
    rightHandOrientationAngled_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientationAngled_.quaternion.x = -0.631;
    rightHandOrientationAngled_.quaternion.y =  0.339;
    rightHandOrientationAngled_.quaternion.z =  0.680;
    rightHandOrientationAngled_.quaternion.w =  0.159;

    rightHandOrientationPerpen_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientationPerpen_.quaternion.x =  0.781;
    rightHandOrientationPerpen_.quaternion.y = -0.155;
    rightHandOrientationPerpen_.quaternion.z = -0.605;
    rightHandOrientationPerpen_.quaternion.w = -0.020;

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftPalm", VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner("rightPalm", VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
}

solar_panel_handle_grabber::~solar_panel_handle_grabber()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}


geometry_msgs::QuaternionStamped solar_panel_handle_grabber::leftHandOrientation() const
{
    return leftHandOrientationAngled_;
}

void solar_panel_handle_grabber::setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation)
{
    leftHandOrientationAngled_ = leftHandOrientation;
}
geometry_msgs::QuaternionStamped solar_panel_handle_grabber::rightHandOrientation() const
{
    return rightHandOrientationAngled_;
}

void solar_panel_handle_grabber::setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation)
{
    rightHandOrientationAngled_ = rightHandOrientation;
}


void solar_panel_handle_grabber::grasp_handles(armSide side, const geometry_msgs::Pose &goal, float executionTime)
{

    const std::vector<float>* seed;
    std::string palmFrame;
    float palmToFingerOffset;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.04;
    }
    else {
        seed = &rightShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.04;
    }


    ROS_INFO("opening grippers");
    gripper_.openGripper(side);

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
    ROS_INFO("Closing grippers");
    gripper_.closeGripper(side);
    ros::Duration(0.3).sleep();
}
