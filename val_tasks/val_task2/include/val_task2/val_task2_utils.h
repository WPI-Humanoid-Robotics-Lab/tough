#pragma once

#include <ros/ros.h>
#include <srcsim/Task.h>
#include <srcsim/Harness.h>
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
#include <val_footstep/RobotWalker.h>
#include <val_task2/cable_detector.h>
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_moveit_planners/val_cartesian_planner.h"
#include "ros/package.h"
#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "val_task2/val_solar_panel_detector.h"
#include <std_msgs/Int8.h>
#include <math.h>
#include <mutex>
#include <navigation_common/map_generator.h>

enum class CLEAR_BOX_CLOUD{
    WAIST_UP=0,
    LARGE_BOX=1,
    FULL_BOX=2
};

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
    RobotWalker *walk_;
    // robot state
    RobotStateInformer *current_state_;
    ros::Publisher reset_pointcloud_pub, reset_map_pub, pause_pointcloud_pub , clearbox_pointcloud_pub , task2_log_pub_,clear_pose_map;

    ros::Subscriber task_status_sub_,detach_harness;

    cartesianPlanner* left_arm_planner_;
    cartesianPlanner* right_arm_planner_;
    wholebodyManipulation* wholebody_controller_;

    CableDetector* cable_detector_;
    srcsim::Task taskMsg;
    srcsim::Harness harnessMsg;

    int current_checkpoint_;
    float table_height_;

    //before walking
    //used for rotating panel
    const std::vector<float> leftNearChestPalmDown_    = {-1.70, -1.04, 1.39, -1.85, 1.50, 0, 0}; //0 -1.70  -1.04  1.39  -1.85  1.50  0  0
    const std::vector<float> rightNearChestPalmDown_   = {-1.70,  1.04, 1.39,  1.85, 1.50, 0, 0}; // 1 -1.7  1.04  1.39  1.85  1.50  0  0
    //used for picking up the panel
    const std::vector<float> leftNearChestPalmUp_    = {-1.7, -1.04, 1.39, -1.85, -1.50, 0, 0};  // 0 -1.5  -1.04  1.6  -1.85  -1.50  0  0
    const std::vector<float> rightNearChestPalmUp_   = {-1.70, 1.04, 1.39,  1.85, -1.50, 0, 0}; //1 -1.70  1.04  1.39  1.85  -1.50  0  0

    const std::vector<float> leftSeedNonGraspingHand_  = {0.21, -1.16, 0.0, -1.07, 1.52, 0, 0};  // 0 0.21  -1.16  0.0  -1.07  1.52  0  0
    const std::vector<float> rightSeedNonGraspingHand_ = {0.21,  1.16, 0.0,  1.07, 1.52, 0, 0}; // 1  0.21  1.16  0.0  1.07  1.52  0  0
    //while walking
    const std::vector<float> leftShoulderSeedPanelGraspWalk_  = {-0.38,-1.38, 1.3,-1.1,-0.26, 0.0, -0.3}; // 0 -0.38  -1.5  1.3  -1.23  -0.26  0.0  -0.2
    const std::vector<float> rightShoulderSeedPanelGraspWalk_ = {-0.38, 1.38, 1.3, 1.1, -0.26, 0.0, 0.3}; // 1 -0.38  1.49  1.3  0.7  -0.26  0.0  0.2


    // panel placement poses
    //const std::vector<float> leftPanelPlacementUpPose1_  = {-1.5, -1.4, 1.39, -0.9, 1.10, -0.5, 0}; // 0 -1.5  -1.4  1.39  -0.9  1.10  -0.5  0
    //const std::vector<float> leftPanelPlacementDownPose1_= {-1.2, -1.4, 1.39, -0.9, 1.10, -0.5, 0}; // 0 -1.2  -1.4  1.39  -0.9  1.10  -0.5  0
    const std::vector<float> leftPanelPlacementUpPose1_ = {-1.9, -1.4, 1.39, -0.9, 1.10, -0.5, 0};// 0 -1.9  -1.4  1.39  -0.9  1.10  -0.5  0
    const std::vector<float> leftPanelPlacementDownPose1_ = {-1.3, -1.4, 1.39, -0.9, 1.10, -0.5, 0};

    //const std::vector<float> rightPanelPlacementUpPose1_  = {-1.5, 1.4, 1.39, 0.9, 1.10, 0.5, 0}; // 1 -1.5  1.4  1.39  0.9  1.10  0.5  0
    //const std::vector<float> rightPanelPlacementDownPose1_= {-1.2, 1.4, 1.39, 0.9, 1.10, 0.5, 0}; // 1 -1.2  1.4  1.39  0.9  1.10  0.5  0

    const std::vector<float> rightPanelPlacementUpPose1_ = {-1.9, 1.4, 1.39, 0.9, 1.10, 0.5, 0}; // 1 -1.9  1.4  1.39  0.9  1.10  0.5  0
    const std::vector<float> rightPanelPlacementDownPose1_ = {-1.3, 1.4, 1.39, 0.9, 1.10, 0.5, 0};

    const std::vector<float> leftPanelPlacementUpPose2_  = {-1.5, -1.4, 1.39, -0.9, -1.10, 0.5, 0}; // 0 -1.5  -1.4  1.39  -0.9  -1.10  0.5  0
    const std::vector<float> leftPanelPlacementDownPose2_= {-1.2, -1.4, 1.39, -0.9, -1.10, 0.5, 0.4}; // 0 -1.2  -1.4  1.39  -0.9  -1.10  0.5  0.4

    const std::vector<float> rightPanelPlacementUpPose2_  = {-1.5, 1.4, 1.39, 0.9, -1.10, -0.5, 0}; // 1 -1.5  1.4  1.39  0.9  -1.10  -0.5  0
    const std::vector<float> rightPanelPlacementDownPose2_= {-1.2, 1.4, 1.39, 0.9, -1.10, -0.5, -0.4};// 1 -1.2  1.4  1.39  0.9  -1.10  -0.5  -0.4

    const std::vector<float> leftPanelPlacementSupport1_  = {-0.66, -1.4, 1.2, -1.49, 1.29, 0, 0.26}; // 0 -0.66  -1.4  1.2  -1.49  1.29  0  0.26
    const std::vector<float> leftPanelPlacementSupport2_  = {-0.66, -1.4, 0.75,-1.49, 1.29, 0, 0.26}; // 0 -0.66  -1.4  0.75  -1.49  1.29  0  0.26

    const std::vector<float> rightPanelPlacementSupport1_ = {-0.66, 1.4, 1.2, 1.49, 1.29, 0, 0.26}; // 1 -0.66  1.4  1.2  1.49  1.29  0  0.26
    const std::vector<float> rightPanelPlacementSupport2_ = {-0.66, 1.4, 0.75, 1.49, 1.29, 0, 0.26};// 1 -0.66  1.4  0.75  1.49  1.29  0  0.26

    // cable in hand seed point
    const std::vector<float> righCableInHandSeed_ = {-0.81,1.11,0.65,1.14,-0.26,-0.19,0.30};// -0.81 1.11 0.65 1.14 -0.26 -0.19 0.30

    // Gripper commands
    const std::vector<double> leftHandGrasp_          = {1.2, -0.6, -0.77, -0.9, -0.9};
    const std::vector<double> rightHandGrasp_         = {1.2,  0.6,  0.77,  0.9,  0.9};

    //Swapping sides of bag
    std::vector<armTrajectory::armJointData> reOrientPanelTrajLeft_;
    std::vector<armTrajectory::armJointData> reOrientPanelTrajRight_;

    void taskStatusCB(const srcsim::Task &msg);
    void isDetachedCB(const srcsim::Harness &harnessMsg);

    std::string logFile;
    std::mutex mtx_;
    nav_msgs::OccupancyGrid map_;
    void mapUpdateCB(const nav_msgs::OccupancyGrid &msg);
    ros::Subscriber mapUpdaterSub_;
    bool isHarnessDetached;

public:
    const std::string TEXT_RED="\033[0;31m";
    const std::string TEXT_GREEN = "\033[0;32m";
    const std::string TEXT_NC=  "\033[0m";

    task2Utils(ros::NodeHandle nh);
    ~task2Utils();
    bool afterPanelGraspPose(const armSide side, bool isRotationRequired);
    bool isPointOnWalkway(float x, float y);
    void movePanelToWalkSafePose(const armSide side, bool isRotationRequired);
    bool isPanelPicked(const armSide side);
    void placePanel(const armSide graspingHand, bool isPanelRotated);
    void rotatePanel(const armSide graspingHand);
    void raisePanel(const armSide graspingHand);
    void clearPointCloud();
    void clearMap();
    void pausePointCloud();
    void resumePointCloud();
    void clearCurrentPoseMap();
    void clearBoxPointCloud(CLEAR_BOX_CLOUD state);
    void reOrientTowardsGoal(geometry_msgs::Point goal_point, float offset=0.0);
    void reOrientTowardsCable(geometry_msgs::Pose cablePose, geometry_msgs::Pose panelPose);
    int getCurrentCheckpoint() const;
    bool shakeTest(const armSide graspingHand);
    bool pushDeployedPanel();

    boost::posix_time::ptime timeNow;
    bool isCableOnTable(geometry_msgs::Pose &cable_pose);
    bool isCableInHand(armSide side);
    bool isCableTouchingSocket();
    geometry_msgs::Pose grasping_hand(armSide &side, geometry_msgs::Pose handle_pose);
    bool isRotationReq(armSide side, geometry_msgs::Point handle_coordinates,geometry_msgs::Point button_coordinates);
    bool checkpoint_init();
    void taskLogPub(std::string data);
    bool planWholeBodyMotion(armSide side, std::vector<geometry_msgs::Pose> waypoints);
};


