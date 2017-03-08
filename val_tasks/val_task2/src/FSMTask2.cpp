#include <iostream>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH


FSM(Robot)
{
    //This are the states
    FSM_STATES
    {
      walking_rover,
      walking_finish,
      walking_solararray,
      walking_cable,

      searching_rover,
      searching_panel,
      searching_cable,
      searching_solararray,
      searching_finish,
      searching_button,

      grasping_cable,
      grasping_panel,
      grasping_button,

      placing_solarpanel,
      plugging_cable
    }

    FSM_START(searching_rover)

    FSM_BGN
    {
        FSM_STATE(searching_rover)
        {
            FSM_CALL_TASK(search_rover)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(walking_rover))
            }
        }
        FSM_STATE(walking_rover)
        {
            FSM_CALL_TASK(walk_rover)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CLOSE_TO_OBJECT", FSM_NEXT(searching_panel))
            }
        }
        FSM_STATE(searching_panel)
        {

            FSM_CALL_TASK(search_panel)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(grasping_panel))
            }
        }
        FSM_STATE(grasping_panel)
        {

            FSM_CALL_TASK(grasp_panel)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GRABBED", FSM_NEXT(searching_solararray))
            }
        }
        FSM_STATE(searching_solararray)
        {

            FSM_CALL_TASK(search_solararray)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(walking_solararray))
            }
        }
        FSM_STATE(walking_solararray)
        {

            FSM_CALL_TASK(walking_solararray)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CLOSE_TO_OBJECT", FSM_NEXT(searching_cable))
            }
        }

        FSM_STATE(searching_cable)
        {

            FSM_CALL_TASK(search_cable)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(walking_cable))
            }
        }
        FSM_STATE(walking_cable)
        {

            FSM_CALL_TASK(walk_cable)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CLOSE_TO_OBJECT", FSM_NEXT(placing_solarpanel))
            }
        }
        FSM_STATE(placing_solarpanel)
        {

            FSM_CALL_TASK(place_solarpanel)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/PLACED", FSM_NEXT(searching_button))
            }
        }
        FSM_STATE(searching_button)
        {

            FSM_CALL_TASK(search_button)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(grasping_button))
            }
        }
        FSM_STATE(grasping_button)
        {

            FSM_CALL_TASK(grasp_button)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(grasping_cable))
            }
        }

        FSM_STATE(grasping_cable)
        {

            FSM_CALL_TASK(grasp_cable)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GRABBED", FSM_NEXT(plugging_cable))
            }
        }
        FSM_STATE(plugging_cable)
        {

            FSM_CALL_TASK(plug_cable)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GRABBED", FSM_NEXT(searching_finish))
            }
        }
        FSM_STATE(searching_finish)
        {
            FSM_CALL_TASK(search_finish)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(walking_finish))
            }
        }
        FSM_STATE(walking_finish)
        {
            FSM_CALL_TASK(walk_finish)

            FSM_TRANSITIONS
            {
                //FSM_ON_EVENT("/STOPPED", FSM_NEXT(stop))
            }
        }
    }
    FSM_END
}


// This functions are called based on the remaping in the main.
// when ever a action is published one of these functions will be called
decision_making::TaskResult searchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    string msg = "searching..." + name;
    //ROS_INFO(msg);
    //eventQueue.riseEvent("/SEARCH_TIMEOUT");
    return TaskResult::SUCCESS();
}


decision_making::TaskResult walkTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    string msg = "walking..." + name;
    //eventQueue.riseEvent("/WALK_TIMEOUT");
    return TaskResult::SUCCESS();
}


decision_making::TaskResult graspTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    string msg = "grasping..." + name;
    //ROS_INFO(msg);
    //eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}


decision_making::TaskResult plugTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    string msg = "pluging..." + name;
    //ROS_INFO(msg);
    //eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}



decision_making::TaskResult placeTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    string msg = "placing..." + name;
    //ROS_INFO(msg);
    //eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult stopTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    string msg = "placing..." + name;
    //ROS_INFO(msg);
    //eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "FSMTask2");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nodeHandle;
    RosEventQueue eventQueue;

    // All the like transations are mapped to the same function
    // the function will decide what to do based on the input string

    // All the searching fucntion
    LocalTasks::registrate("search_rover", searchTask);
    LocalTasks::registrate("search_panel", searchTask);
    LocalTasks::registrate("search_cable", searchTask);
    LocalTasks::registrate("search_solararray", searchTask);
    LocalTasks::registrate("searching_finish", searchTask);
    LocalTasks::registrate("searching_button", searchTask);

    // All the walking functions
    LocalTasks::registrate("walk_rover", walkTask);
    LocalTasks::registrate("walk_finish", walkTask);
    LocalTasks::registrate("walk_solararray", walkTask);

    // All the grasping functions
    LocalTasks::registrate("grasp_cable", graspTask);
    LocalTasks::registrate("grasp_panel", graspTask);
    LocalTasks::registrate("grasp_button", graspTask);

    // Unquie calls
    LocalTasks::registrate("placing_solarpanel", placeTask);
    LocalTasks::registrate("plugging_cable", plugTask);
    LocalTasks::registrate("stop", stopTask);



	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting mission...");
	FsmRobot(NULL, &eventQueue);

	spinner.stop();

	return 0;
}
