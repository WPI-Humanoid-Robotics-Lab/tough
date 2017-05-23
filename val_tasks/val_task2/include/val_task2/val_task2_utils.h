#pragma once

#include <ros/ros.h>
#include <srcsim/Satellite.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <val_common/val_common_names.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/val_head_navigation.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_footstep/ValkyrieWalker.h>

class task2Utils {
private:
    ros::NodeHandle nh_;
    // chest controller
    chestTrajectory* chest_controller_;
    //pelvis controller
    pelvisTrajectory* pelvis_controller_;
    //head controller
    HeadTrajectory* head_controller_;
    //grippers
    gripperControl* gripper_controller_;
    // arm
    armTrajectory* arm_controller_;
    // walker class
    ValkyrieWalker *walk_;




    // Need to edit
    const std::vector<float> leftShoulderSeedPanelGraspStatic_ = {-1.15, -1.04, 1.39, -1.69, 1.89, 0, 0};
    const std::vector<float> leftShoulderSeedPanelGraspWalk_ = {-0.04, -1.16, 0.12, -0.94, 1.13, -0.01, 0.08};

    //before walking
    const std::vector<float> rightShoulderSeedPanelGraspStatic_ = {-1.15, 1.04, 1.39, 1.69, 1.89, 0, 0};
    //while walking
    const std::vector<float> rightShoulderSeedPanelGraspWalk_ = {-0.04, 1.16, 0.12, 0.94, 1.13, 0.01, -0.08};

public:
    task2Utils(ros::NodeHandle nh);
    void afterPanelGraspPose(const armSide side);
    ~task2Utils();

};
