#include <tough_controller_interface/pelvis_control_interface.h>

int PelvisControlInterface::pelvis_id_ = -1;

PelvisControlInterface::PelvisControlInterface(ros::NodeHandle nh):nh_(nh)
{
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    pelvisHeightPublisher_ = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/pelvis_height_trajectory",1,true);
                                      //    ihmc_msgs/PelvisHeightTrajectoryRosMessage
    state_informer_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
}

PelvisControlInterface::~PelvisControlInterface()
{

}

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

    PelvisControlInterface::pelvis_id_--;
    msg.unique_id = PelvisControlInterface::pelvis_id_;


    // publish the message
    pelvisHeightPublisher_.publish(msg);
}

bool PelvisControlInterface::controlPelvisMessage(ihmc_msgs::PelvisHeightTrajectoryRosMessage msg){

    this->pelvisHeightPublisher_.publish(msg);
    PelvisControlInterface::pelvis_id_--;
    msg.unique_id = PelvisControlInterface::pelvis_id_;
}
