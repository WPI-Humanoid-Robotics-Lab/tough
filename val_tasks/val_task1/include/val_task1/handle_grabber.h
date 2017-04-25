#ifndef HANDLE_GRABBER_H
#define HANDLE_GRABBER_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <val_control/val_arm_navigation.h>
#include <val_control/val_gripper_control.h>

#define Y_OFFSET 0.05

class handle_grabber{
public:
    handle_grabber(ros::NodeHandle n);
    ~handle_grabber();
   void grab_handle(const armSide side, const geometry_msgs::Point &goal, float executionTime=2.0f);

   geometry_msgs::QuaternionStamped leftHandOrientation() const;
   void setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation);

   geometry_msgs::QuaternionStamped rightHandOrientation() const;
   void setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation);

   bool transformQuaternionToWorld(const geometry_msgs::QuaternionStamped &qt_in, geometry_msgs::QuaternionStamped &qt_out, std::string target_frame=VAL_COMMON_NAMES::WORLD_TF);

private:
   ros::NodeHandle nh_;
   tf::TransformListener listener_;
   armTrajectory armTraj_;
   gripperControl gripper_;

   geometry_msgs::QuaternionStamped leftHandOrientation_ ;
   geometry_msgs::QuaternionStamped rightHandOrientation_;

   const std::vector<float> leftShoulderSeed_ = {-0.23, -0.72, 0.65, -1.51, 2.77, 0.0, 0.0};
   const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65, 1.51, 2.77, 0.0, 0.0};

};

#endif // HANDLE_GRABBER_H
