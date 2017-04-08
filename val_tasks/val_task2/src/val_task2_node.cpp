#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include <ros/ros.h>
#include<functional>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <val_task2/val_task2.h>

using namespace std;

#define foreach BOOST_FOREACH

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task2");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nh;
    RosEventQueue*q = new RosEventQueue();

    // task2 object
    valTask2 task2(nh); // = new valTask2(nh);
    ROS_INFO("Preparing task2...");

    // Register all the functions
    LocalTasks::registrate("STATE_INIT", std::bind(&valTask2::initTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_ROVER", std::bind(&valTask2::detectRoverTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_ROVER", std::bind(&valTask2::walkToRoverTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_SOLAR_PANEL",std::bind(&valTask2::detectPanelTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ORIENT_TO_SOLAR_PANEL",std::bind(&valTask2::orientPanelTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PICK_SOLAR_PANEL",std::bind(&valTask2::pickPanelTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_SOLAR_ARRAY", std::bind(&valTask2::detectSolarArrayTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_SOLAR_ARRAY", std::bind(&valTask2::walkSolarArrayTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PLACE_SOLAR_PANEL_ON_GROUND", std::bind(&valTask2::placePanelTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_DEPLOY_PANEL_BUTTON", std::bind(&valTask2::detectButtonTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DEPLOY_SOLAR_PANEL", std::bind(&valTask2::deployPanelTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_POWER_CABLE", std::bind(&valTask2::dtectCableTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PICKUP_POWER_CABLE", std::bind(&valTask2::pickCableTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PLUGIN_POWER_CABLE", std::bind(&valTask2::plugCableTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_FINISH", std::bind(&valTask2::detectfinishBoxTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_FINISH", std::bind(&valTask2::walkToFinishTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_END", std::bind(&valTask2::endTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ERROR", std::bind(&valTask2::errorTask, task2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting Task 2");
    Fsmval_task2(NULL, q, "val_task2");

    spinner.stop();

    return 0;
}
