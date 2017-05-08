#include <val_task2/button_press.h>

button_press::button_press(ros::NodeHandle& nh):nh_(nh), armTraj_(nh), gripper_(nh), bd_(nh)
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

    /* Top Grip */
    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientation_.quaternion.x = -0.576;
    rightHandOrientation_.quaternion.y = 0.397;
    rightHandOrientation_.quaternion.z = 0.632;
    rightHandOrientation_.quaternion.w = 0.332;

}

button_press::~button_press()
{

}


geometry_msgs::QuaternionStamped button_press::leftHandOrientation() const
{
    return leftHandOrientation_;
}

void button_press::setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation)
{
    leftHandOrientation_ = leftHandOrientation;
}

geometry_msgs::QuaternionStamped button_press::rightHandOrientation() const
{
    return rightHandOrientation_;
}

void button_press::setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation)
{
    rightHandOrientation_ = rightHandOrientation;
}


void button_press::grasp_button(const armSide side, geometry_msgs::Point &goal, float executionTime)
{
    bd_.findButtons(goal);

    // closing grippers
    ROS_INFO("closing grippers");
    gripper_.closeGripper(side);

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    std::vector< std::vector<float> > armData;
    armData.push_back(side == armSide::LEFT ? leftShoulderSeed_ : rightShoulderSeed_);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*1.5).sleep();
    armData.clear();

    ROS_INFO("Moving at an intermidate point before goal");
    geometry_msgs::Pose pt;
    pt.position = goal;
    pt.position.z += 0.1;
    geometry_msgs::QuaternionStamped temp  = side == armSide::LEFT ? leftHandOrientation_ : rightHandOrientation_;

    current_state_->transformQuaternion(temp, temp);
    pt.orientation = temp.quaternion;

    armTraj_.moveArmInTaskSpace(side, pt, executionTime);
    ros::Duration(executionTime*1.4).sleep();

    ROS_INFO("Moving towards goal");
    pt.position = goal;
    pt.position.z += 0.02;

    armTraj_.moveArmInTaskSpace(side, pt, executionTime);
    ros::Duration(executionTime).sleep();

    ROS_INFO("Moving at an intermidate point after goal");
    pt.position = goal;
    pt.position.z += 0.1;

    current_state_->transformQuaternion(temp, temp);
    pt.orientation = temp.quaternion;

    armTraj_.moveArmInTaskSpace(side, pt, executionTime);
    ros::Duration(executionTime*1.4).sleep();

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    armData.push_back(side == armSide::LEFT ? leftShoulderSeed_ : rightShoulderSeed_);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*1.5).sleep();

    ROS_INFO("Opening grippers");
    gripper_.openGripper(side);
    ros::Duration(executionTime*1.5).sleep();

}



