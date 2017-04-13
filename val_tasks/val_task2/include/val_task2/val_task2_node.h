# pragma once

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
#include <dynamic_reconfigure/server.h>
#include "/home/sumanth/indigo_ws/src/space_robotics_challenge/val_tasks/val_task2/cfg/cpp/val_task2/task2_parametersConfig.h"


class task2Node {

public:

    task2Node(ros::NodeHandle nh);
    ~task2Node();

    void paramUpdateCallback(val_task2::task2_parametersConfig &config, uint32_t level);
    void registerStateMethods(void);

private:
    ros::NodeHandle nh_;

    // task2 object
    valTask2* task2_;
};
