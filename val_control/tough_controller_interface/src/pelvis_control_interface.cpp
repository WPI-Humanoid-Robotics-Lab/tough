#include <tough_controller_interface/pelvis_control_interface.h>
#include <val_common/val_common_names.h>

int pelvisTrajectory::pelvis_id = -1;

pelvisTrajectory::pelvisTrajectory(ros::NodeHandle nh):nh_(nh)
{
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    pelvisHeightPublisher = nh_.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/pelvis_height_trajectory",1,true);
}

pelvisTrajectory::~pelvisTrajectory()
{

}

void pelvisTrajectory::controlPelvisHeight(float height)
{

    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;
    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    tf::TransformListener listener_;
    tf::StampedTransform transform;
    ros::Duration(0.2).sleep();
    try
    {
        ros::Time zero = ros::Time(0);
        listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF, zero, ros::Duration(10.0));
        listener_.lookupTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::L_FOOT_TF, zero, transform);

    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }

    p.position = height + transform.getOrigin().getZ();
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
