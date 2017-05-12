#include <val_task2/button_press.h>

button_press::button_press(ros::NodeHandle& nh):nh_(nh), armTraj_(nh), gripper_(nh), bd_(nh_), walk_(nh,1.0,1.0,0.18)
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

    //frame_ = "buttonFrame";

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


bool button_press::grasp_button(const armSide side, geometry_msgs::Point &goal, float executionTime)
{
//    geometry_msgs::PointStamped geom_point;
//    tf::TransformListener listener;
//    geom_point.point.x = goal.x;
//    geom_point.point.y = goal.y;
//    geom_point.point.z = goal.z;
//    geom_point.header.frame_id = frame_;
//    try
//    {
//        listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, frame_, ros::Time(0), ros::Duration(3.0));
//        listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point, geom_point);
//    }
//    catch (tf::TransformException ex){
//        ROS_ERROR("%s",ex.what());
//        return false;
//    }

//    goal.x = geom_point.point.x;
//    goal.y = geom_point.point.y;
//    goal.z = geom_point.point.z;

    // closing grippers
    ROS_INFO("closing grippers");
    gripper_.controlGripper(side, side == armSide::LEFT ?  leftGripperSeed_ : rightGripperSeed_);

    //inorder to avoid collision of other arm with panel
    ROS_INFO_STREAM("Setting Shoulder Roll for " << (side == armSide::LEFT ? RIGHT : LEFT) << " Side");
    std::vector< std::vector<float> > armData;
    armData.push_back(side == armSide::LEFT ?  rightShoulderSeed_ : leftShoulderSeed_);

    armTraj_.moveArmJoints(side == armSide::LEFT ? RIGHT : LEFT, armData, executionTime);
    ros::Duration(executionTime*1.5).sleep();
    armData.clear();

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
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
    pt.position.z += 0.2;

    current_state_->transformQuaternion(temp, temp);
    pt.orientation = temp.quaternion;
    armTraj_.moveArmInTaskSpace(side, pt, 0);
    ros::Duration(executionTime).sleep();

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    armData.push_back(side == armSide::LEFT ? leftShoulderSeed_ : rightShoulderSeed_);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*1.5).sleep();

    ROS_INFO("Opening grippers");
    gripper_.openGripper(side);
    ros::Duration(executionTime*1.5).sleep();

    ROS_INFO_STREAM("Default " << side << " Side");
    armTraj_.moveToDefaultPose(side);
    ros::Duration(executionTime*1.5).sleep();

    ROS_INFO_STREAM("Default " << (side == armSide::LEFT ? RIGHT : LEFT) << " Side");
    armTraj_.moveToDefaultPose(side == armSide::LEFT ? RIGHT : LEFT);
    ros::Duration(executionTime*1.5).sleep();

    return true;
}

void button_press::getButton( geometry_msgs::Point &goal)
{
    bd_.findButtons(goal);
//    tf::Transform transform;
//    tf::Quaternion q;
//    transform.setOrigin( tf::Vector3(goal.x , goal.y, goal.z) );
//    q.setRPY(0, 0, 0);
//    transform.setRotation(q);
//    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), VAL_COMMON_NAMES::WORLD_TF, frame_));
//    ROS_INFO("Button Transformed");
}

void button_press::orientRobot ()
{
    geometry_msgs::Pose2D goal;
    goal.x = 0.0;
    goal.y = 0.0;
    goal.theta =-1.57;
    ROS_INFO("About to walk");
    walk_.turn(RIGHT);
}
