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
    // robot state
    RobotStateInformer *current_state_;
    ros::Publisher reset_pointcloud_pub ;
    ros::Publisher pause_pointcloud_pub ;


    // Need to edit
    const std::vector<float> leftSeedGraspingHand_ = {-1.40, -1.04, 1.39, -1.85, -1.10, 0, 0};
    const std::vector<float> leftSeedNonGraspingHand_ = {0.21, -1.16, 0.0, -1.07, 1.52, 0, 0};
    const std::vector<float> leftShoulderSeedPanelGraspWalk_ = {-0.04, -1.16, 0.12, -0.94, 1.13, -0.01, 0.08};

    //before walking
    const std::vector<float> rightSeedGraspingHand = {-1.40, 1.04, 1.39, 1.85, -1.10, 0, 0};
    const std::vector<float> rightSeedNonGraspingHand_ = {0.21, 1.16, 0.0, 1.07, 1.52, 0, 0};
    //while walking
    const std::vector<float> rightShoulderSeedPanelGraspWalk_ = {-0.04, 1.16, 0.12, 0.94, 1.13, 0.01, -0.08};

public:
    task2Utils(ros::NodeHandle nh);
    void afterPanelGraspPose(const armSide side);
    void movePanelToWalkSafePose(const armSide side);
    bool isPanelPicked(const armSide side);
    void clearPointCloud();
    void pausePointCloud();
    void resumePointCloud();
    ~task2Utils();

};
