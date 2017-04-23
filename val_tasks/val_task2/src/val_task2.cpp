#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <val_task2/val_task2.h>

using namespace std;

#define foreach BOOST_FOREACH

ValkyrieWalker *valTask2::walker_ = NULL;
geometry_msgs::Pose2D valTask2::panel_walk_goal_;

// constructor and destrcutor
valTask2::valTask2(ros::NodeHandle nh):
    nh_(nh)
{
    // object for the valkyrie walker
    walker_ = new ValkyrieWalker(nh_, 0.5, 0.5, 0, 0.18);
}

// destructor
valTask2::~valTask2(){

}

bool valTask2::preemptiveWait(double ms, decision_making::EventQueue& queue) {
    for (int i = 0; i < 100 && !queue.isTerminated(); i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

    return queue.isTerminated();
}

// This functions are called based on the remaping in the main.
// when ever a action is published one of these functions will be called
decision_making::TaskResult valTask2::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // the state transition can happen from an event externally or can be geenerated here
    //!!!!! depends on the developer and use case

    // generate the event
    eventQueue.riseEvent("/INIT_SUCESSFUL");

    // wait infinetly until an external even occurs
    //    while(!preemptiveWait(1000, eventQueue)){
    //           ROS_INFO("waiting for transition");
    //    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/DETECTED_ROVER");

    return TaskResult::SUCCESS();
}


decision_making::TaskResult valTask2::walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // for storing the fail count
    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
//    ret = walker_->walkToGoal(panel_walk_goal_, false);

//    // if executing stay in the same state
//    if (ret == MOVE_EXECUTING)
//    {
//         eventQueue.riseEvent("/WALK_EXECUTING");
//    }
//    // if finished sucessfully
//    else if (ret == MOVE_SUCESS)
//    {
//       eventQueue.riseEvent("/REACHED_ROVER");
//    }
//    // if failed for more than 5 times, go to error state
//    else if (fail_count > 5)
//    {
//        // reset the fail count
//        fail_count = 0;
//        eventQueue.riseEvent("/WALK_FAILED");
//    }
//    // if failed retry detecting the panel and then walk
//    // also handles MOVE_FAILED
//    else
//    {
//        // increment the fail count
//        fail_count++;
//        eventQueue.riseEvent("/WALK_RETRY");
//    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::orientPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::dtectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/WALK_TO_END");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

    return TaskResult::SUCCESS();
}


// setter and getter methods
geometry_msgs::Pose2D valTask2::getPanelWalkGoal()
{
    return panel_walk_goal_;
}

void valTask2::setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    panel_walk_goal_ = panel_walk_goal;
}
