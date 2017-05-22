#ifndef BUTTON_PRESS_H
#define BUTTON_PRESS_H
#include <val_task2/button_detector.h>
#include <val_control/val_arm_navigation.h>
#include <val_control/val_gripper_control.h>
#include <val_footstep/ValkyrieWalker.h>
#include <val_common/val_common_defines.h>
#include <val_common/val_common_names.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_control/val_wholebody_manipulation.h"
#include "val_control/val_chest_navigation.h"


class button_press
{
    armTrajectory armTraj_;
    ros::NodeHandle nh_;
    ButtonDetector bd_;
    tf::TransformListener listener_;
    gripperControl gripper_;
    RobotStateInformer *current_state_;
    ValkyrieWalker walk_;
    geometry_msgs::QuaternionStamped leftHandOrientation_ ;
    geometry_msgs::QuaternionStamped rightHandOrientation_;

    //    const std::vector<float> leftShoulderSeed_ = {-1.06 ,-0.77, 0.70, -1.12 ,1.96, 0.0, 0.0};
    //    const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65 , 1.51, 2.77, 0.0, 0.0};

    // Not in use currently
    const std::vector<float> leftShoulderSeed_ = {-0.23 ,-1.16, -0.09, -1.39 ,1.09, 0.02, -0.06};
    const std::vector<float> rightShoulderSeed_ = {-0.28, 0.99, 0.12 , 1.49, 1.03, 0.0, 0.0};
    cartesianPlanner* right_arm_planner_;
    cartesianPlanner* left_arm_planner_;
    wholebodyManipulation* wholebody_controller_;
    chestTrajectory * chest_controller_;

public:

    bool pressButton(const armSide side, geometry_msgs::Point &goal, float executionTime=2.0f);
    geometry_msgs::QuaternionStamped leftHandOrientation() const;
    geometry_msgs::QuaternionStamped rightHandOrientation() const;
    void getButtonPosition( geometry_msgs::Point &goal);
    button_press(ros::NodeHandle &);
    ~button_press();

};

#endif // BUTTON_PRESS_H
