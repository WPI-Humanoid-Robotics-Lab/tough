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

bool preemptiveWait(double ms, decision_making::EventQueue& queue) {
  for (int i = 0; i < 100 && !queue.isTerminated(); i++)
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

  return queue.isTerminated();
}

// state machine state executions
decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
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

decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}


decision_making::TaskResult walkToControlPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/REACHED_PANEL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult detectHandleCenterTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult adjustArmTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/YAW_CORRECTION_SUCESSFUL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/WALK_TO_END");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task1");
  ros_decision_making_init(argc, argv);
  ros::NodeHandle nh;
  RosEventQueue* q = new RosEventQueue();

  ROS_INFO("Preparing Task1...");

  // register the api's for states
  LocalTasks::registrate("STATE_INIT", initTask);
  LocalTasks::registrate("STATE_DETECT_PANEL", detectPanelTask);
  LocalTasks::registrate("STATE_WALK_TO_CONTROL", walkToControlPanelTask);
  LocalTasks::registrate("STATE_DETECT_HANDLE_CENTER", detectHandleCenterTask);
  LocalTasks::registrate("STATE_ADJUST_ARMS", adjustArmTask);
  LocalTasks::registrate("STATE_CORRECT_PITCH", controlPitchTask);
  LocalTasks::registrate("STATE_CORRECT_YAW", controlYawTask);
  LocalTasks::registrate("STATE_DETECT_FINISH", detectfinishBoxTask);
  LocalTasks::registrate("STATE_WALK_TO_FINISH", walkToFinishTask);
  LocalTasks::registrate("END_STATE", endTask);
  LocalTasks::registrate("STATE_ERROR", errorTask);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Starting Task1");
  Fsmval_task1(NULL, q, "val_task1");

  spinner.stop();

  return 0;
}
