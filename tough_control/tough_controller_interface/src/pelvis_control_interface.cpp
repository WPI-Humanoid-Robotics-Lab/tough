#include <tough_controller_interface/pelvis_control_interface.h>

int PelvisControlInterface::pelvis_id_ = -1;

PelvisControlInterface::PelvisControlInterface(ros::NodeHandle nh):nh_(nh)
{
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    pelvisHeightPublisher_ = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/pelvis_height_trajectory",1,true);
    state_informer_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
}

PelvisControlInterface::~PelvisControlInterface()
{

}

void PelvisControlInterface::controlPelvisHeight(float height)
{

    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
    ihmc_msgs::TrajectoryPoint1DRosMessage p;
//    tf::TransformListener listener_;
//    tf::StampedTransform transform;
//    ros::Duration(0.2).sleep();
    geometry_msgs::Pose foot_pose;
    state_informer_->getCurrentPose(rd_->getLeftFootFrameName(), foot_pose);

//    try
//    {
//        ros::Time zero = ros::Time(0);
//        listener_.waitForTransform(TOUGH_COMMON_NAMES::WORLD_TF, TOUGH_COMMON_NAMES::PELVIS_TF, zero, ros::Duration(10.0));
//        listener_.lookupTransform(TOUGH_COMMON_NAMES::WORLD_TF, TOUGH_COMMON_NAMES::L_FOOT_TF, zero, transform);

//    }
//    catch (tf::TransformException ex)
//    {
//        ROS_WARN("%s",ex.what());
//        return;
//    }

    p.position = height + foot_pose.position.z;
    p.velocity = 0.5;
    p.time = 0.0;

    msg.trajectory_points.clear();
    msg.trajectory_points.push_back(p);
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
