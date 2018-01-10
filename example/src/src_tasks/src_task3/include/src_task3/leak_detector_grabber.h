#ifndef LEAK_DETECTOR_GRAB_H
#define LEAK_DETECTOR_GRAB_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_controller_interface/robot_state.h>
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"
#include <src_task_common/val_task_common_utils.h>
#include <tf/transform_datatypes.h>
#include <src_task3/val_task3_utils.h>

class leakDetectorGrabber{

public:
    leakDetectorGrabber(ros::NodeHandle nh);
    ~leakDetectorGrabber();

    void graspDetector(geometry_msgs::Pose goal, float executionTime=2.0f);

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    ArmControlInterface armTraj_;
    GripperControlInterface gripper_;
    RobotStateInformer *current_state_;
    RobotDescription *rd_;
    CartesianPlanner *right_arm_planner_;
    CartesianPlanner *left_arm_planner_;
    WholebodyControlInterface wholebody_controller_;
    geometry_msgs::QuaternionStamped leftHandOrientation_ ;
    geometry_msgs::QuaternionStamped rightHandOrientation_;

    ros::Publisher marker_pub_;
    task3Utils task3_utils_;
    void pubPoseArrow(const geometry_msgs::Pose &pose,
                      const std::string &ns,
                      const int id,
                      const float r,
                      const float g,
                      const float b) const;
    void pubPoseArrow(const geometry_msgs::Pose &pose,
                      const std::string &ns,
                      const int id = 0) const { pubPoseArrow(pose, ns, id, 0.f, 0.5f, 0.5f); };
    void pubPoseArrow(const geometry_msgs::Pose &pose,
                      const std::string &ns,
                      const float r,
                      const float g,
                      const float b) const { pubPoseArrow(pose, ns, 0, r, g, b); }

    geometry_msgs::Pose getGraspGoal(const RobotSide &side, const geometry_msgs::Pose &user_goal) const;
    geometry_msgs::Pose getReachGoal(const RobotSide &side, const geometry_msgs::Pose &grasp_goal) const;

    float getStandingOffset(const RobotSide side, const geometry_msgs::Pose user_goal) const;
};

#endif // LEAK_DETECTOR_GRASP_H
