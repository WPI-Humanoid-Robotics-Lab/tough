#include <ros/ros.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_robot");
    ros::NodeHandle nh;

    // initializing objects
    armTrajectory armTraj(nh);
    chestTrajectory chestTraj(nh);
    pelvisTrajectory pelvisTraj(nh);

    armTraj.moveToZeroPose(armSide::LEFT);
    ros::Duration(0.3).sleep();
    armTraj.moveToZeroPose(armSide::RIGHT);
    ros::Duration(1.5).sleep();

    pelvisTraj.controlPelvisHeight(0.9);
    ros::Duration(1.5).sleep();
    chestTraj.controlChest(2,2,2);
    ros::Duration(1).sleep();

    armTraj.moveToDefaultPose(armSide::LEFT);
    ros::Duration(0.3).sleep();
    armTraj.moveToDefaultPose(armSide::RIGHT);
    ros::Duration(1.5).sleep();

}

