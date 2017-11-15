# pragma once

#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include<functional>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <val_task3/val_task3.h>
#include <dynamic_reconfigure/server.h>
#include "val_task3/task3_parametersConfig.h"

class task3Node{

public:
    task3Node(ros::NodeHandle nh);
    ~task3Node();

    void paramUpdateCallback(val_task3::task3_parametersConfig &config, uint32_t level);
    void registerStateMethods();

private:
    ros::NodeHandle nh_;

    // task3 object
    valTask3* task3_;
};
