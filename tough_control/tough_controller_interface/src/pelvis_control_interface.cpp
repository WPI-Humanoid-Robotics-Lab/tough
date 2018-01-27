#include <tough_controller_interface/pelvis_control_interface.h>

PelvisControlInterface::PelvisControlInterface(ros::NodeHandle nh):ToughControllerInterface(nh)
{
    pelvisHeightPublisher_ = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>(control_topic_prefix_ + "/pelvis_height_trajectory",1,true);
                                      //    ihmc_msgs/PelvisHeightTrajectoryRosMessage
}

PelvisControlInterface::~PelvisControlInterface()
{
    pelvisHeightPublisher_.shutdown();
}

/**
 * @brief PelvisControlInterface::controlPelvisHeight controls the height of the pelvis with respect to the ground
 * @param height is the height in meters
 */
void PelvisControlInterface::controlPelvisHeight(float height)
{
    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
    ihmc_msgs::EuclideanTrajectoryPointRosMessage p;

    geometry_msgs::Pose foot_pose;
    state_informer_->getCurrentPose(rd_->getLeftFootFrameName(), foot_pose);

    ihmc_msgs::FrameInformationRosMessage reference_frame;
    reference_frame.trajectory_reference_frame_id = -102;   //Pelvis frame
    reference_frame.data_reference_frame_id = -102;//Pelvis frame
    msg.frame_information = reference_frame;
    p.position.z = height + foot_pose.position.z;
    p.linear_velocity.z = 0.5;
    p.time = 0.0;

    msg.taskspace_trajectory_points.clear();
    msg.taskspace_trajectory_points.push_back(p);
    msg.use_custom_control_frame = false;


    // publish the message
    publishPelvisMessage(msg);
}

void PelvisControlInterface::publishPelvisMessage(const ihmc_msgs::PelvisHeightTrajectoryRosMessage &msg) const{
    this->pelvisHeightPublisher_.publish(msg);
}
