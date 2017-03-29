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

bool preemptiveWait(double ms, decision_making::EventQueue& queue) {
  for (int i = 0; i < 100 && !queue.isTerminated(); i++)
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

  return queue.isTerminated();
}

// This functions are called based on the remaping in the main.
// when ever a action is published one of these functions will be called
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

decision_making::TaskResult detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  return TaskResult::SUCCESS();
}


decision_making::TaskResult walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/REACHED_ROVER");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult orientPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult dtectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

  return TaskResult::SUCCESS();
}

decision_making::TaskResult plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // generate the event
  eventQueue.riseEvent("/REACHED_ROVER");

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

    ros::init(argc, argv, "task2");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nh;
    RosEventQueue*q = new RosEventQueue();

    ROS_INFO("Preparing Task2...");

    // All the searching fucntion
    LocalTasks::registrate("STATE_INIT", initTask);
    LocalTasks::registrate("STATE_DETECT_ROVER", detectRoverTask);
    LocalTasks::registrate("STATE_WALK_TO_ROVER", walkToRoverTask);
    LocalTasks::registrate("STATE_DETECT_SOLAR_PANEL", detectPanelTask);
    LocalTasks::registrate("STATE_ORIENT_TO_SOLAR_PANEL", orientPanelTask);
    LocalTasks::registrate("STATE_PICK_SOLAR_PANEL", pickPanelTask);
    LocalTasks::registrate("STATE_DETECT_SOLAR_ARRAY", detectSolarArrayTask);
    LocalTasks::registrate("STATE_WALK_TO_SOLAR_ARRAY", walkSolarArrayTask);
    LocalTasks::registrate("STATE_PLACE_SOLAR_PANEL_ON_GROUND", placePanelTask);
    LocalTasks::registrate("STATE_DETECT_DEPLOY_PANEL_BUTTON", detectButtonTask);
    LocalTasks::registrate("STATE_DEPLOY_SOLAR_PANEL", deployPanelTask);
    LocalTasks::registrate("STATE_DETECT_POWER_CABLE", dtectCableTask);
    LocalTasks::registrate("STATE_PICKUP_POWER_CABLE", pickCableTask);
    LocalTasks::registrate("STATE_PLUGIN_POWER_CABLE", plugCableTask);
    LocalTasks::registrate("STATE_DETECT_FINISH", detectfinishBoxTask);
    LocalTasks::registrate("STATE_WALK_TO_FINISH", walkToFinishTask);
    LocalTasks::registrate("STATE_END", endTask);
    LocalTasks::registrate("STATE_ERROR", errorTask);


    ros::AsyncSpinner spinner(1);
	spinner.start();

    ROS_INFO("Starting Task 2");
    Fsmval_task2(NULL, q, "val_task2");

	spinner.stop();

	return 0;
}
