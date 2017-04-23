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
#include <val_task1/val_task1.h>


using namespace std;

#define foreach BOOST_FOREACH

ValkyrieWalker *valTask1::walker_ = NULL;
geometry_msgs::Pose2D valTask1::panel_walk_goal_;

// constructor and destrcutor
valTask1::valTask1(ros::NodeHandle nh):
    nh_(nh)
{
    // object for the valkyrie walker
    walker_ = new ValkyrieWalker(nh_, 0.5, 0.5, 0, 0.18);
}

// destructor
valTask1::~valTask1(){

}

bool valTask1::preemptiveWait(double ms, decision_making::EventQueue& queue) {
  for (int i = 0; i < 100 && !queue.isTerminated(); i++)
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

  return queue.isTerminated();
}

// state machine state executions
decision_making::TaskResult valTask1::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // the state transition can happen from an event externally or can be geenerated here
  //!!!!! depends on the developer and use case

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  // wait infinetly until an external even occurs
  while(!preemptiveWait(1000, eventQueue)){
     ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}


decision_making::TaskResult valTask1::walkToControlPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/REACHED_PANEL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectHandleCenterTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::adjustArmTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/YAW_CORRECTION_SUCESSFUL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/WALK_TO_END");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

// setter and getter methods
geometry_msgs::Pose2D valTask1::getPanelWalkGoal()
{
    return panel_walk_goal_;
}

void valTask1::setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    panel_walk_goal_ = panel_walk_goal;
}



