#pragma once

// !!!!! important other wise created collision with std
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

#include "val_footstep/ValkyrieWalker.h"
#include "val_task_common/val_walk_tracker.h"
#include "val_task_common/panel_detection.h"
#include "val_control/val_chest_navigation.h"
#include "val_control/val_pelvis_navigation.h"
#include "val_control/val_head_navigation.h"
#include "val_control/val_gripper_control.h"
#include "val_control/robot_state.h"
#include "val_task2/val_rover_detection.h"

using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(val_task2)

class valTask2 {

    private:
    ros::NodeHandle nh_;
    // default constructor
    valTask2(ros::NodeHandle nh);

    // object for the walker api
    ValkyrieWalker* walker_;
    // object for tracking robot walk
    walkTracking* walk_track_;
    // panel detection object
    PanelDetector* panel_detector_;
    //Rover detector
    RoverDetector* rover_detector_;

    // chest controller
    chestTrajectory* chest_controller_;
    //pelvis controller
    pelvisTrajectory* pelvis_controller_;
    //head controller
    HeadTrajectory* head_controller_;
    //grippers
    gripperControl* gripper_controller_;
    //robot state informer
    RobotStateInformer* robot_state_;

    ros::Subscriber occupancy_grid_sub_;
    unsigned int map_update_count_;
    void occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg);

    // goal location for the panel
    geometry_msgs::Pose2D panel_walk_goal_;
    // goal location for the panel
    geometry_msgs::Pose2D rover_walk_goal_;

    void initControllers();
    void initDetectors();

    static valTask2* currentObject;
    public:


    // default destructor
    ~valTask2();
    static valTask2* getValTask2(ros::NodeHandle nh);
    bool preemptiveWait(double ms, decision_making::EventQueue& queue);
    decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult orientPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult dtectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    geometry_msgs::Pose2D getPanelWalkGoal();
    void setRoverWalkGoal(const geometry_msgs::Pose2D &rover_walk_goal);
    void setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal);
};
