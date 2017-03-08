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
        walking_panel,
        walking_cable,
        walking_solararray,
        walking_finish,

        searching_rover,
        searching_panel,
        searching_cable,
        searching_solararray,
        searching_finish,

        grasping_cable,
        grasping_panel

        carrying_panel,

        pressing_button
    }

    FSM_START(searching)

    FSM_BGN
    {
        FSM_STATE(searching_rover)
        {
            FSM_CALL_TASK(search)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/FOUND", FSM_NEXT(walking))
            }
        }
        FSM_STATE(walking)
        {
            FSM_CALL_TASK(walk)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/CLOSE_TO_OBJECT", FSM_NEXT(stopped))
            }
        }
        FSM_STATE(stopped)
        {

            FSM_CALL_TASK(stop)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/GRABBED", FSM_NEXT(searching))
            }
        }
    }
    FSM_END
}


/*************************************************************************************************
*** Task implementations
**************************************************************************************************/

decision_making::TaskResult searchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("searching...");

    eventQueue.riseEvent("/SEARCH_TIMEOUT");
    return TaskResult::SUCCESS();
}


decision_making::TaskResult walkTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("walk...");

    eventQueue.riseEvent("/WALK_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult stopTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO("stop...");

    eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "FSM_test");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nodeHandle;
    RosEventQueue eventQueue;

    LocalTasks::registrate("search", searchTask);
    LocalTasks::registrate("walk", walkTask);
    LocalTasks::registrate("stop", stopTask);


	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting mission...");
	FsmRobot(NULL, &eventQueue);

	spinner.stop();

	return 0;
}
