# pragma once

#include <ros/ros.h>

class moveTracking {

   private:
    ros::NodeHandle nh_;
    ros::Subscriber tf_sub_;

public:
   moveTracking(ros::NodeHandle nh);
   ~moveTracking();
};
