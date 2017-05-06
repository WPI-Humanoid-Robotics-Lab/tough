#include <val_task1/handle_grabber.h>
#include <stdlib.h>
#include <stdio.h>


handle_grabber::handle_grabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
  leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
  leftHandOrientation_.quaternion.x = 0.604;
  leftHandOrientation_.quaternion.y = 0.434;
  leftHandOrientation_.quaternion.z = -0.583;
  leftHandOrientation_.quaternion.w = 0.326;

  rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
  rightHandOrientation_.quaternion.x = -0.576;
  rightHandOrientation_.quaternion.y = 0.397;
  rightHandOrientation_.quaternion.z = 0.632;
  rightHandOrientation_.quaternion.w = 0.332;
}

handle_grabber::~handle_grabber()
{

}


geometry_msgs::QuaternionStamped handle_grabber::leftHandOrientation() const
{
  return leftHandOrientation_;
}

void handle_grabber::setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation)
{
  leftHandOrientation_ = leftHandOrientation;
}
geometry_msgs::QuaternionStamped handle_grabber::rightHandOrientation() const
{
  return rightHandOrientation_;
}

void handle_grabber::setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation)
{
  rightHandOrientation_ = rightHandOrientation;
}

bool handle_grabber::transformQuaternionToWorld(const geometry_msgs::QuaternionStamped &qt_in, geometry_msgs::QuaternionStamped &qt_out, std::string target_frame)
{
  try{

    listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
    listener_.transformQuaternion(target_frame, qt_in, qt_out);

  }
  catch (tf::TransformException ex){
    ROS_WARN("%s",ex.what());
    ros::spinOnce();
    return false;
  }
  return true;
}



void handle_grabber::grab_handle(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{

  // opening grippers
  ROS_INFO("opening grippers");
  gripper_.openGripper(side);

  //move shoulder roll outwards
  ROS_INFO("Setting shoulder roll");
  std::vector< std::vector<float> > armData;
  armData.push_back(side == armSide::LEFT ? leftShoulderSeed_ : rightShoulderSeed_);

  armTraj_.moveArmJoints(side, armData, executionTime);
  ros::Duration(executionTime*1.2).sleep();

  //move arm to given point with known orientation and higher z
  ROS_INFO("Moving at an intermidate point before goal");
  geometry_msgs::Pose pt;
  pt.position = goal;
  pt.position.z += 0.2;
  geometry_msgs::QuaternionStamped temp  = side == armSide::LEFT ? leftHandOrientation_ : rightHandOrientation_;

  transformQuaternionToWorld(temp, temp);
  pt.orientation = temp.quaternion;

  armTraj_.moveArmInTaskSpace(side, pt, executionTime);
  ros::Duration(executionTime*2).sleep();

    //move arm to final position with known orientation
  ROS_INFO("Moving towards goal");
  pt.position = goal;
  armTraj_.moveArmInTaskSpace(side, pt, executionTime);
  ros::Duration(executionTime*2).sleep();


  ROS_INFO("Nudge forward to align palm");
  armTraj_.nudgeArmLocal(side, direction::FRONT);
  ros::Duration(1).sleep();

  ROS_INFO("Nudge down to align palm");
  armTraj_.nudgeArmLocal(side, direction::UP); // go up to go down.
  ros::Duration(1).sleep();

  ROS_INFO("Closing grippers");
  gripper_.closeGripper(side);

}
