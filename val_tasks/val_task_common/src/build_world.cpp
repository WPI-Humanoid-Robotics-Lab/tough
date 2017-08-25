#include <ros/ros.h>
//#include "val_footstep/RobotWalker.h"
#include <val_controllers/val_head_navigation.h>
//#include <val_task2/val_task2_utils.h>

// This node makes the robot rotate 360 degrees and moves head around to generate map and point cloud
int main(int argc, char **argv)
{
    ros::init(argc, argv, "build_world");
    ros::NodeHandle nh;

    /*
    RobotWalker walk(nh, 1.0,1.0,0);
    HeadTrajectory head_controller(nh);
    task2Utils utils; // only req to print logs

    walk.walk_rotate(1.57);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20,-10);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20, 10);
    ros::Duration(2).sleep();

    walk.walk_rotate(1.57);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20,-10);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20, 10);
    ros::Duration(2).sleep();

    walk.walk_rotate(1.57);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20,-10);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20, 10);
    ros::Duration(2).sleep();


    walk.walk_rotate(1.57);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20,-10);
    ros::Duration(2).sleep();
    head_controller.moveHead(0,20, 10);
    ros::Duration(2).sleep();
    utils.taskLogPub("build world complete"); */

    return 0;
}


