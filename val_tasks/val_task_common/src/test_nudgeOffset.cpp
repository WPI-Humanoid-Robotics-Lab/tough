#include <val_controllers/val_arm_navigation.h>
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_moveit_planners/val_cartesian_planner.h"
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

void taskLog(std::string data, std_msgs::String &publishData){

    publishData.data.clear();
    publishData.data = data;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nudgeOffset");
    ros::NodeHandle nh;
    std_msgs::String publishData;
    ROS_INFO("Moving the arms");
    armTrajectory armTraj(nh);
    ros::Publisher taskPB = nh.advertise<std_msgs::String>("/field/log",10);
    wholebodyManipulation wholeBodyController(nh);
    cartesianPlanner right_arm_planner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    cartesianPlanner left_arm_planner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    moveit_msgs::RobotTrajectory traj;

    ros::Duration(1).sleep();
    armSide side;
    std::vector<geometry_msgs::Pose> waypoint;
    geometry_msgs::Pose pose;
    if(argc == 6){
        side = std::atoi(argv[1]) == 0 ? armSide::LEFT : armSide::RIGHT;
        if(std::atoi(argv[2]) ==0) armTraj.nudgeArmLocal(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]),pose);
        else armTraj.nudgeArmPelvis(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]),pose);

        waypoint.clear();
        waypoint.push_back(pose);
        if(side ==armSide::RIGHT)
        {
            if (right_arm_planner.getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                ROS_INFO("right arm whole body msg executed");
                taskLog("test_nudgeOffset : right arm whole body msg executed",publishData);
                taskPB.publish(publishData);
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            }
        }
        else
        {
            if (left_arm_planner.getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                ROS_INFO("left arm whole body msg executed");
                taskLog("test_nudgeOffset : left arm whole body msg executed",publishData);
                taskPB.publish(publishData);
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            }

        }

    }
    else
    {
        ROS_INFO(" 0 - local | 1 - pelvis, invalid input");
    }
    ros::spinOnce();
    return 0;
}


