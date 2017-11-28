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
#include <src_task1/val_task1.h>
#include <dynamic_reconfigure/server.h>
#include "src_task1/task1_parametersConfig.h"

class task1Node {

public:

    task1Node(ros::NodeHandle nh);
    ~task1Node();

    void paramUpdateCallback(src_task1::task1_parametersConfig &config, uint32_t level);
    void registerStateMethods(void);

private:
    ros::NodeHandle nh_;

    // task1 object
    valTask1* task1_;
};
