#pragma once

#include <ros/ros.h>
#include<srcsim/Satellite.h>

class task1Utils {
private:
    ros::NodeHandle nh_;
    ros::Subscriber satellite_sub_;
    srcsim::Satellite msg_;

    void satelliteMsgCB (const srcsim::Satellite &msg);

    bool isPitchCorrectNow(void);
    bool isYawCorrectNow(void);
    bool isPitchCompleted(void);
    bool isYawCompleted(void);
    double getPitchDiff (void);
    double getYawDiff (void);

public:
    task1Utils(ros::NodeHandle nh);
    ~task1Utils();
};
