#include <tough_controller_interface/arm_control_interface.h>
#include "tough_controller_interface/wholebody_control_interface.h"
#include "tough_moveit_planners/tough_cartesian_planner.h"
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_nudgeOffset");
    ros::NodeHandle nh;


    ros::Publisher log_pub = nh.advertise<std_msgs::String>(VAL_COMMON_NAMES::LOG_TOPIC, 10);
    const auto log_msg = [&log_pub](const std::string &str) {
        std_msgs::String msg;
        msg.data = ros::this_node::getName() + ": " + str;
        log_pub.publish(msg);
        ROS_INFO("%s", msg.data.c_str());
    };

    // wait a reasonable amount of time for the subscriber to connect
    ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
    while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until) {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
    }

    std_msgs::String publishData;
    log_msg("starting planners");
    ArmControlInterface armTraj(nh);
    wholebodyManipulation wholeBodyController(nh);
    CartesianPlanner right_arm_planner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    CartesianPlanner left_arm_planner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    moveit_msgs::RobotTrajectory traj;

    ros::Duration(1).sleep();
    RobotSide side;
    std::vector<geometry_msgs::Pose> waypoint;
    geometry_msgs::Pose pose;
    if(argc == 6){
        side = std::atoi(argv[1]) == 0 ? RobotSide::LEFT : RobotSide::RIGHT;
        std::string side_str = (side == RobotSide::LEFT ? "left" : "right");
        if(std::atoi(argv[2]) ==0) {
            log_msg("Nudging " + side_str + " arm by given amount using nudgeArmLocal");
            armTraj.nudgeArmLocal(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]),pose);
        } else {
            log_msg("Nudging " + side_str + " arm by given amount using nudgeArmPelvis");
            armTraj.nudgeArmPelvis(side, std::atof(argv[3]),std::atof(argv[4]),std::atof(argv[5]),pose);
        }

        waypoint.clear();
        waypoint.push_back(pose);
        if(side ==RobotSide::RIGHT)
        {
            if (right_arm_planner.getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                log_msg("right arm whole body msg executing");
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            } else {
                log_msg("right arm whole body msg planning was unsuccessful");
            }
        }
        else
        {
            if (left_arm_planner.getTrajFromCartPoints(waypoint, traj, false)> 0.98){
                log_msg("left arm whole body msg executing");
                wholeBodyController.compileMsg(side, traj.joint_trajectory);
            } else {
                log_msg("left arm whole body msg planning was unsuccessful");
            }

        }

    }
    else
    {
        log_msg("expected arguments: side [0: left, 1: right], nudge_type [0: local, 1: pelvis], x, y, z. Exiting.");
        return -1;
    }
    ros::spinOnce();

    log_msg("node finished, trajectory may still be executing");
    return 0;
}


