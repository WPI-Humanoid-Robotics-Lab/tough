# pragma once

#include <ros/ros.h>
#include <val_task3/val_task3_utils.h>
#include <srcsim/Leak.h>

class leakDetector{
private:
    ros::NodeHandle nh_;
    ros::Subscriber leak_sb_;

    double leak_value_;

    void generateSearchWayPoints (void);

public:
    leakDetector(ros::NodeHandle nh);
    ~leakDetector();

    void leakMsgCB(const srcsim::Leak &leakmsg);
    double getLeakValue() const;
    void setLeakValue(double getLeakValue);
};
