#include <val_task2/solar_panel_grasp.h>
#include <stdlib.h>
#include <stdio.h>


solar_panel_handle_grabber::solar_panel_handle_grabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
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

    /* Top Grip */
    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientation_.quaternion.x = -0.576;
    rightHandOrientation_.quaternion.y = 0.397;
    rightHandOrientation_.quaternion.z = 0.632;
    rightHandOrientation_.quaternion.w = 0.332;
}

solar_panel_handle_grabber::~solar_panel_handle_grabber()
{

}


geometry_msgs::QuaternionStamped solar_panel_handle_grabber::leftHandOrientation() const
{
    return leftHandOrientation_;
}

void solar_panel_handle_grabber::setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation)
{
    leftHandOrientation_ = leftHandOrientation;
}
geometry_msgs::QuaternionStamped solar_panel_handle_grabber::rightHandOrientation() const
{
    return rightHandOrientation_;
}

void solar_panel_handle_grabber::setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation)
{
    rightHandOrientation_ = rightHandOrientation;
}


void solar_panel_handle_grabber::grasp_handles(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{

    // opening grippers
    ROS_INFO("opening grippers");
    gripper_.openGripper(side);

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    std::vector< std::vector<float> > armData;
    armData.push_back(side == armSide::LEFT ? leftShoulderSeed_ : rightShoulderSeed_);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();

    //move arm to given point with known orientation and higher z
    ROS_INFO("Moving at an intermidate point before goal");
    geometry_msgs::Pose pt;
    pt.position = goal;
    pt.position.z += 0.1;
    geometry_msgs::QuaternionStamped temp  = side == armSide::LEFT ? leftHandOrientation_ : rightHandOrientation_;

    current_state_->transformQuaternion(temp, temp);
    pt.orientation = temp.quaternion;

    armTraj_.moveArmInTaskSpace(side, pt, executionTime);
    ros::Duration(executionTime*1.4).sleep();

    //move arm to final position with known orientation
    geometry_msgs::Pose finalGoal;
    geometry_msgs::Point finalPoint;

    current_state_->transformPoint(goal,finalPoint, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    finalPoint.x += 0.1;
    finalPoint.z -= 0.1;

    //transform that point back to world frame
    current_state_->transformPoint(finalPoint, finalPoint, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
    finalGoal.position = finalPoint;

    ROS_INFO("Moving towards goal");
    pt.position = finalPoint;

    armTraj_.moveArmInTaskSpace(side, pt, executionTime);
    ros::Duration(executionTime).sleep();

    ROS_INFO("Closing grippers");
    gripper_.closeGripper(side);

}
