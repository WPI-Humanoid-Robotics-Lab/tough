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


class task3Node{

public:
    task3Node(ros::NodeHandle nh);
    ~task3Node();

    void registerStateMethods();

private:
    ros::NodeHandle nh_;
    valTask3* task3_;

};
