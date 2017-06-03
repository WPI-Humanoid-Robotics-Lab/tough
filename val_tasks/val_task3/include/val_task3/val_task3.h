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


using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(val_task3)

class valTask3{

    private:
    ros::NodeHandle nh_;
    valTask3(ros::NodeHandle nh);

    task3Utils*         task3_utils_;
    ValkyrieWalker*     walker_;
    walkTracking*       walk_track_;
    chestTrajectory*    chest_controller_;
    pelvisTrajectory*   pelvis_controller_;
    HeadTrajectory*     head_controller_;
    gripperControl*     gripper_controller_;
    RobotStateInformer* robot_state_;

    //map and occupancy grid
    ros::Subscriber occupancy_grid_sub_;
    unsigned int map_update_count_;
    void occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg);

    geometry_msgs::Pose2D leak_detector_walk_goal_;
    geometry_msgs::Pose2D leak_repair_walk_goal_;
    geometry_msgs::Pose2D leak_position_walk_goal_;

    static valTask3* currentObject;

    public:

    ~valTask3();

    static valTask3* getValTask3(ros::NodeHandle nh);

    bool preemptiveWait(double ms, decision_making::EventQueue& queue);

    decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult climbStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
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

};
