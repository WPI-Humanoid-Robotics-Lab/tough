#pragma once

/*******************************************************
 * !!!!! important other wise creates collision with std
 * *****************************************************/
#define DISABLE_DECISION_MAKING_LOG true

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <val_footstep/ValkyrieWalker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <val_task3/val_task3_utils.h>

#include "val_task_common/val_task_common_utils.h"
#include "val_footstep/ValkyrieWalker.h"
#include "val_task_common/val_walk_tracker.h"
#include "val_task_common/panel_detection.h"
#include "val_controllers/val_chest_navigation.h"
#include "val_controllers/val_pelvis_navigation.h"
#include "val_controllers/val_head_navigation.h"
#include "val_controllers/val_gripper_control.h"
#include "val_controllers/robot_state.h"
#include <val_controllers/val_wholebody_manipulation.h>
#include <mutex>
#include <val_task3/stair_detector_2.h>
#include <val_task3/climb_stairs.h>
#include <val_task3/door_valve_detector.h>
#include <val_task3/door_opener.h>

using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(val_task3)

class valTask3{

    private:
    ros::NodeHandle nh_;
    valTask3(ros::NodeHandle nh);

    ValkyrieWalker*     walker_;
    walkTracking*       walk_track_;
    chestTrajectory*    chest_controller_;
    pelvisTrajectory*   pelvis_controller_;
    HeadTrajectory*     head_controller_;
    gripperControl*     gripper_controller_;
    // arm
    armTrajectory*      arm_controller_;
    wholebodyManipulation* wholebody_controller_;
    RobotStateInformer* robot_state_;
    task3Utils*         task3_utils_;

    //detectors
    stair_detector_2*   stair_detector_;
    DoorValvedetector*  door_valve_detcetor_;

    climbStairs*        climb_stairs_;
    doorOpener*         door_opener_;

    //map and occupancy grid
    ros::Subscriber occupancy_grid_sub_;
    unsigned int map_update_count_;
    void occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg);

    // private vairiables storing the pose and locations
    geometry_msgs::Pose2D stair_detect_walk_pose_;
    geometry_msgs::Pose   handle_center_;
    geometry_msgs::Pose2D table_walk_pose_;
    geometry_msgs::Point  leak_detector_loc_;
    geometry_msgs::Pose2D leak_wall_pose_;
    geometry_msgs::Point  leak_loc_;
    geometry_msgs::Point  repair_tool_loc_;
    geometry_msgs::Pose2D finish_box_pose_;

    static valTask3 *currentObject;

    void resetRobotToDefaults(int arm_pose=1);

    public:

    const std::string TEXT_RED="\033[0;31m";
    const std::string TEXT_GREEN = "\033[0;32m";
    const std::string TEXT_NC=  "\033[0m";

    ~valTask3();

    static valTask3* getValTask3(ros::NodeHandle nh);

    bool preemptiveWait(double ms, decision_making::EventQueue& queue);

    decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult climbStepsTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectDoorHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult openDoorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkthroughDoorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectLeakToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToLeakToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickLeakTool(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToDetectLeak(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectLeakTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectRepairToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToRepairToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickRepairTool(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToLeakTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult leakRepairTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    geometry_msgs::Pose2D stair_detect_walk_pose() const;

    // setter getter api's
    void setStairDetectWalkPose(const geometry_msgs::Pose2D &stair_detect_walk_pose);
    void setHandleCenter(const geometry_msgs::Pose &handle_center);
    void setTableWalkPose(const geometry_msgs::Pose2D &table_walk_pose);
    void setLeakDetectorLoc(const geometry_msgs::Point &leak_detector_loc);
    void setLeakWallPose(const geometry_msgs::Pose2D &leak_wall_pose);
    void setLeakLoc(const geometry_msgs::Point &leak_loc);
    void setRepairToolLoc(const geometry_msgs::Point &repair_tool_loc);
    void setFinishBoxPose(const geometry_msgs::Pose2D &finish_box_pose);
};
