#include <tough_controller_interface/pelvis_control_interface.h>

PelvisControlInterface::PelvisControlInterface(ros::NodeHandle nh):ToughControllerInterface(nh)
{
    pelvisHeightPublisher_ = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>(control_topic_prefix_ + "/pelvis_height_trajectory",1,true);
}

PelvisControlInterface::~PelvisControlInterface()
{
    pelvisHeightPublisher_.shutdown();
}

void PelvisControlInterface::controlPelvisHeight(float height)
{

    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
    ihmc_msgs::TrajectoryPoint1DRosMessage p;

    geometry_msgs::Pose foot_pose;
    state_informer_->getCurrentPose(rd_->getLeftFootFrameName(), foot_pose);

    p.position = height + foot_pose.position.z;
    p.velocity = 0.5;
    p.time = 0.0;

    msg.trajectory_points.clear();
    msg.trajectory_points.push_back(p);
    msg.unique_id = id_++;

    // publish the message
    publishPelvisMessage(msg);
}

void PelvisControlInterface::publishPelvisMessage(const ihmc_msgs::PelvisHeightTrajectoryRosMessage &msg) const{
    this->pelvisHeightPublisher_.publish(msg);
}
