#include <val_controllers/val_arm_navigation.h>
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_moveit_planners/val_cartesian_planner.h"
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nudgeOffset");
    ros::NodeHandle nh;
    ROS_INFO("Moving the arms");
    armTrajectory armTraj(nh);
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
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            }
        }
        else
        {
            if (left_arm_planner.getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                ROS_INFO("left arm whole body msg executed");
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


