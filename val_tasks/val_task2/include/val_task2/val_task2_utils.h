#pragma once

#include <ros/ros.h>
#include <srcsim/Task.h>
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
#include <val_task2/cable_detector.h>
#include "ros/package.h"
#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"

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
    ros::Publisher clearbox_pointcloud_pub ;

    ros::Subscriber task_status_sub_;

    CableDetector* cable_detector_;
    srcsim::Task taskMsg;
    int current_checkpoint_;
    float table_height_;

    //before walking
    const std::vector<float> leftNearChestPalmDown_    = {-1.70, -1.04, 1.39, -1.85, -1.10, 0, 0};
    const std::vector<float> leftNearChestPalmUp_    = {-1.70, -1.04, 1.39, -1.85, 1.10, 0, 0};
    const std::vector<float> leftSeedNonGraspingHand_ = {0.21, -1.16, 0.0, -1.07, 1.52, 0, 0};
    //while walking
    const std::vector<float> leftShoulderSeedPanelGraspWalk_ = {-0.38, -1.29, 0.99, -1.35, -0.26, 0.0, 0.0};


    //before walking
    const std::vector<float> rightNearChestPalmDown_     = {-1.70, 1.04, 1.39, 1.85, -1.10, 0, 0};
    const std::vector<float> rightNearChestPalmUp_     = {-1.70, 1.04, 1.39, 1.85, 1.10, 0, 0};
    const std::vector<float> rightSeedNonGraspingHand_ = {0.21, 1.16, 0.0, 1.07, 1.52, 0, 0};
    //while walking
    const std::vector<float> rightShoulderSeedPanelGraspWalk_ = {-0.38, 1.29, 0.99, 1.35, -0.26, 0.0, 0.0};

    // panel placement poses
    const std::vector<float> leftPanelPlacementUpPose1_  = {-1.5, -1.4, 1.39, -0.9, -1.10, 0.5, 0};
    const std::vector<float> leftPanelPlacementDownPose1_= {-1.2, -1.4, 1.39, -0.9, -1.10, 0.5, 0.4};

    const std::vector<float> leftPanelPlacementUpPose2_  = {-1.5, -1.4, 1.39, -0.9, 1.10, -0.5, 0};
    const std::vector<float> leftPanelPlacementDownPose2_= {-1.2, -1.4, 1.39, -0.9, 1.10, -0.5, 0};

    const std::vector<float> leftPanelPlacementSupport1_  = {-0.66, -1.4, 1.2, -1.49, 1.29, 0, 0.26};
    const std::vector<float> leftPanelPlacementSupport2_  = {-0.66, -1.4, 0.75, -1.49, 1.29, 0, 0.26};

    const std::vector<float> rightPanelPlacementUpPose1_  = {-1.5, 1.4, 1.39, 0.9, -1.10, -0.5, 0};
    const std::vector<float> rightPanelPlacementDownPose1_= {-1.2, 1.4, 1.39, 0.9, -1.10, -0.5, 0.4};

    const std::vector<float> rightPanelPlacementUpPose2_  = {-1.5, 1.4, 1.39, 0.9, 1.10, 0.5, 0};
    const std::vector<float> rightPanelPlacementDownPose2_= {-1.2, 1.4, 1.39, 0.9, 1.10, 0.5, 0};

    const std::vector<float> rightPanelPlacementSupport1_ = {-0.66, 1.4, 1.2, 1.49, 1.29, 0, 0.26};
    const std::vector<float> rightPanelPlacementSupport2_ = {-0.66, 1.4, 0.75, 1.49, 1.29, 0, 0.26};

    // Gripper commands
    const std::vector<double> leftHandGrasp_          = {1.2, -0.6, -0.77, -0.9, -0.9};
    const std::vector<double> rightHandGrasp_         = {1.2,  0.6,  0.77,  0.9,  0.9};

    //Swapping sides of bag
    std::vector<armTrajectory::armJointData> reOrientPanelTraj_;

    void taskStatusCB(const srcsim::Task &msg);

    std::string logFile;

public:
    task2Utils(ros::NodeHandle nh);
    ~task2Utils();
    void afterPanelGraspPose(const armSide side);
    void movePanelToWalkSafePose(const armSide side);
    bool isPanelPicked(const armSide side);
    void moveToPlacePanelPose(const armSide graspingHand, bool rotatePanel);
    void rotatePanel(const armSide graspingHand);
    void clearPointCloud();
    void pausePointCloud();
    void resumePointCloud();
    void clearBoxPointCloud();
    void reOrientTowardsPanel(geometry_msgs::Pose panelPose);
    int getCurrentCheckpoint() const;
    bool shakeTest(const armSide graspingHand);
    boost::posix_time::ptime timeNow;
    bool isCableOnTable(geometry_msgs::Pose &cable_coordinates);
    bool isCableInHand(armSide side);
    bool isCableTouchingSocket();
};
