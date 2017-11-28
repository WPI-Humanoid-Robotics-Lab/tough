#include <tough_control_common/tough_control_common.h>

valControlCommon::valControlCommon(ros::NodeHandle nh): nh_(nh), armTraj(nh), chestTraj(nh), pelvisTraj(nh)

{
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    // set the publisher
    stop_traj_pub_ = nh_.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/stop_all_trajectories",1,true);
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

void valControlCommon::resetRobot()
{
    armTraj.moveToZeroPose(RobotSide::LEFT);
    ros::Duration(0.3).sleep();
    armTraj.moveToZeroPose(RobotSide::RIGHT);
    ros::Duration(1.5).sleep();

    pelvisTraj.controlPelvisHeight(0.9);
    ros::Duration(1.5).sleep();
    chestTraj.controlChest(2,2,2);
    ros::Duration(1).sleep();

    armTraj.moveToDefaultPose(RobotSide::LEFT);
    ros::Duration(0.3).sleep();
    armTraj.moveToDefaultPose(RobotSide::RIGHT);
    ros::Duration(1.5).sleep();
}
