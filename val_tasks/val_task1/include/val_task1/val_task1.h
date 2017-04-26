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
#include <geometry_msgs/Pose2D.h>
#include <val_footstep/ValkyrieWalker.h>
#include <val_task_common/val_walk_tracker.h>
#include <val_task1/panel_detection.h>

using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(val_task1)

class valTask1 {

    private:
    ros::NodeHandle nh_;
    valTask1(ros::NodeHandle nh);
    // object for the walker api
    ValkyrieWalker* walker_;

    // object for tracking robot walk
    walkTracking* walk_track_;

    // panel detection object
    panel_detector* panel_detector_;

    bool isPoseChanged(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new);

    static valTask1 *currentObject;

    public:

    // goal location for the panel
    geometry_msgs::Pose2D panel_walk_goal_;

    // default constructor and destructor
    static valTask1* getValTask1(ros::NodeHandle nh);
    ~valTask1();

    bool preemptiveWait(double ms, decision_making::EventQueue& queue);
    decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToControlPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectHandleCenterTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult adjustArmTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);

    geometry_msgs::Pose2D getPanelWalkGoal();
    void setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal_);
};
