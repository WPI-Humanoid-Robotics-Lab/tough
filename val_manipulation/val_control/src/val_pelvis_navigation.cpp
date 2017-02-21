#include <val_control/val_pelvis_navigation.h>

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
    pelvisTrajectory::pelvis_id--;
    msg.unique_id = pelvisTrajectory::pelvis_id;

    // publish the message
    pelvisHeightPublisher.publish(msg);
}

bool pelvisTrajectory::controlPelvisMessage(ihmc_msgs::PelvisHeightTrajectoryRosMessage msg){

    this->pelvisHeightPublisher.publish(msg);
    pelvisTrajectory::pelvis_id--;
    msg.unique_id = pelvisTrajectory::pelvis_id;
}
