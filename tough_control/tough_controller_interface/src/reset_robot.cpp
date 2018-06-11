#include <ros/ros.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/head_control_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_robot");
    ros::NodeHandle nh;

    // initializing objects
    ArmControlInterface armTraj(nh);
    ChestControlInterface chestTraj(nh);
    PelvisControlInterface pelvisTraj(nh);
    HeadControlInterface head(nh);

    if(argc ==2)
    {

        armTraj.moveToZeroPose(RobotSide::LEFT);
        ros::Duration(0.3).sleep();
        armTraj.moveToZeroPose(RobotSide::RIGHT);
        ros::Duration(1).sleep();

    }


    pelvisTraj.controlPelvisHeight(0.9);
    ros::Duration(1.5).sleep();
    chestTraj.controlChest(2,2,2);
    ros::Duration(1).sleep();

    armTraj.moveToDefaultPose(RobotSide::LEFT);
    ros::Duration(0.3).sleep();
    armTraj.moveToDefaultPose(RobotSide::RIGHT);
    ros::Duration(1).sleep();

    head.moveHead(0, 0, 0, 2);
    ros::Duration(2).sleep();

    ROS_INFO("Robot has been reset");

}

