#include <val_pelvis_navigation.h>

pelvisTrajectory::pelvisTrajectory(ros::NodeHandle nh):nh_(nh)
{
    pelvisHeightPublisher = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/pelvis_height_trajectory",1,true);
}

pelvisTrajectory::~pelvisTrajectory()
{

}

void pelvisTrajectory::controlPelvisHeight(float height)
{

    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
    ihmc_msgs::TrajectoryPoint1DRosMessage p;

    p.position = height;
    p.velocity = 0.5;
    p.time = 0.0;

    msg.trajectory_points.clear();
    msg.trajectory_points.push_back(p);
    msg.unique_id = 13;

    // publish the message
    pelvisHeightPublisher.publish(msg);
}
