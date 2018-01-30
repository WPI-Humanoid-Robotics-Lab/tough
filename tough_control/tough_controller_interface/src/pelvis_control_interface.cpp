#include <tough_controller_interface/pelvis_control_interface.h>

PelvisControlInterface::PelvisControlInterface(ros::NodeHandle nh):ToughControllerInterface(nh)
{

    pelvisHeightPublisher_ = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>(control_topic_prefix_ +"/pelvis_height_trajectory",1,true);
                                      //    ihmc_msgs/PelvisHeightTrajectoryRosMessage
}

PelvisControlInterface::~PelvisControlInterface()
{

}

/**
 * @brief PelvisControlInterface::controlPelvisHeight controls the height of the pelvis with respect to the feet
 * @param height is the height in meters
 */
void PelvisControlInterface::controlPelvisHeight(float height, float duration)
{
    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
    ihmc_msgs::EuclideanTrajectoryPointRosMessage p;

    geometry_msgs::Pose foot_pose;
    state_informer_->getCurrentPose(rd_->getLeftFootFrameName(), foot_pose);

    ihmc_msgs::FrameInformationRosMessage reference_frame;
    reference_frame.trajectory_reference_frame_id = rd_->getPelvisFrameHash();   //Pelvis frame
    reference_frame.data_reference_frame_id = rd_->getPelvisFrameHash();//Pelvis frame
    msg.frame_information = reference_frame;

    p.position.z = height + foot_pose.position.z;
    p.time = duration;

    msg.taskspace_trajectory_points.clear();
    msg.taskspace_trajectory_points.push_back(p);
    msg.use_custom_control_frame = false;

    PelvisControlInterface::id_++;
    msg.unique_id = PelvisControlInterface::id_;

    // publish the message
    pelvisHeightPublisher_.publish(msg);
}

bool PelvisControlInterface::controlPelvisMessage(ihmc_msgs::PelvisHeightTrajectoryRosMessage msg){

    this->pelvisHeightPublisher_.publish(msg);
    PelvisControlInterface::id_++;
    msg.unique_id = PelvisControlInterface::id_;
}
