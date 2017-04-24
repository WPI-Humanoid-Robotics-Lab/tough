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

    // object for the walker api
    static ValkyrieWalker* walker_;

    // object for tracking robot walk
    static walkTracking* walk_track_;

    // panel detection object
    static panel_detector* panel_detector_;

    static bool isPoseChanged(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new);


    public:

    // goal location for the panel
    static geometry_msgs::Pose2D panel_walk_goal_;

    // default constructor and destructor
    valTask1(ros::NodeHandle nh);
    ~valTask1();

    static bool preemptiveWait(double ms, decision_making::EventQueue& queue);
    static decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult walkToControlPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult detectHandleCenterTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult adjustArmTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    static decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);

    static geometry_msgs::Pose2D getPanelWalkGoal();
    static void setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal_);
};
