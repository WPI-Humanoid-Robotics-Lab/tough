#include "val_task2/insert_cable.h"


insertCable::insertCable(ros::NodeHandle n):nh_(n), armTraj_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);

        /* Side Grip */
        leftHandOrientation_.quaternion.x = 0.155;
        leftHandOrientation_.quaternion.y = -0.061;
        leftHandOrientation_.quaternion.z = -0.696;
        leftHandOrientation_.quaternion.w = 0.699;

        /* Side Grip */
        rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
        rightHandOrientation_.quaternion.x = -0.094;
        rightHandOrientation_.quaternion.y = -0.027;
        rightHandOrientation_.quaternion.z = 0.973;
        rightHandOrientation_.quaternion.w = -0.209;

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_ = new wholebodyManipulation(nh_);
    chest_controller_ = new chestTrajectory(nh_);

}

insertCable::~insertCable()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}

void insertCable::insert_cable(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{
    // setting initial values
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    std::string palmFrame;
    float palmToFingerOffset;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        palmToFingerOffset = 0.07;
        finalOrientationStamped = &leftHandOrientation_;
    }
    else {
        seed = &rightShoulderSeed_;
        palmFrame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        palmToFingerOffset = -0.07;
        finalOrientationStamped = &rightHandOrientation_;

    }

    // getting the orientation
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;
    current_state_->transformQuaternion(temp, temp);

    //move arm to given point with known orientation and higher z
    geometry_msgs::Point finalGoal, intermGoal;

    ROS_INFO("Moving at an intermidate point before goal");

    current_state_->transformPoint(goal,finalGoal, VAL_COMMON_NAMES::WORLD_TF, palmFrame);
    intermGoal=finalGoal;

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

    ROS_INFO("Moving chest to zero position");
    chest_controller_->controlChest(0,0,0);

}

