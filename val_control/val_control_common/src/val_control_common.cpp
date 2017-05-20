#include <val_control_common/val_control_common.h>

valControlCommon::valControlCommon(ros::NodeHandle nh): nh_(nh)
{

    // set the publisher
    stop_traj_pub_ = nh_.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/stop_all_trajectories",1,true);
}

valControlCommon::~valControlCommon()
{

}

void valControlCommon::stopAllTrajectories(void)
{
    ihmc_msgs::StopAllTrajectoryRosMessage stop_msg;
    stop_msg.unique_id = -1;

    // send the message
    stop_traj_pub_.publish(stop_msg);

    ros::Duration(1).sleep();
}
