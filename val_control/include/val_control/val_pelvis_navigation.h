#ifndef VAL_PELVIS_NAVIGATION_H
#define VAL_PELVIS_NAVIGATION_H

#include <ihmc_msgs/PelvisHeightTrajectoryRosMessage.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>


class pelvisTrajectory {

private:

    ros::NodeHandle nh_;
    ros::Publisher pelvisHeightPublisher;
    static int pelvis_id;

public:

    pelvisTrajectory(ros::NodeHandle nh);
    ~pelvisTrajectory();
    void controlPelvisHeight(float height);
    bool controlPelvisMessage(ihmc_msgs::PelvisHeightTrajectoryRosMessage msg);
};

#endif
