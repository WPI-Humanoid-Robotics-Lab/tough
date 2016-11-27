#include<val_manipulation/val_arm_navigation_.h>
#include<stdlib.h>
#include <stdio.h>

ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos);
ihmc_msgs::PelvisHeightTrajectoryRosMessage controlPelvisHeight(float height);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_navigation");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher armTrajectoryPublisher = nh.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory", 1,true);
    ros::Publisher handTrajectoryPublisher = nh.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/valkyrie/control/hand_desired_configuration", 1,true);
    ros::Publisher pelvisHeightPublisher = nh.advertise<ihmc_msgs::PelvisHeightTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/pelvis_height_trajectory",1,true);

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;

    //{1.5, -0.4, -1.2, 1.4, 0.0, 0.0, 0.0};
    float BUTTON_PRESS_PREPARE [] ={1.57, -0.1, -1.6, 1.55, 0.0, 0.0, 0.0};
    float BUTTON_PRESS_ACT [] ={1.57, -0.4, -1.6, 1.1, 0.0, 0.0, 0.0};
    float RETRACT_TO_ACTUAL [] ={-0.2, 1.2, 0.7222, 1.5101, 0.0, 0.0, 0.0};
    float TEST [] ={1.57, -0.3, -1.2, 1.4, 0.0, 0.0, 0.0};

    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = arm_traj.RIGHT;
    arm_traj.unique_id = -1;

    arm_traj = appendTrajectoryPoint(arm_traj, 2, BUTTON_PRESS_PREPARE);
    arm_traj = appendTrajectoryPoint(arm_traj, 4, BUTTON_PRESS_ACT);
    arm_traj = appendTrajectoryPoint(arm_traj, 4, RETRACT_TO_ACTUAL);

    //arm_traj = appendTrajectoryPoint(arm_traj, 2, TEST);

    armTrajectoryPublisher.publish(arm_traj);

    ihmc_msgs::HandDesiredConfigurationRosMessage hand_msg;
    hand_msg.robot_side = hand_msg.RIGHT;
    hand_msg.hand_desired_configuration = hand_msg.CLOSE;
    hand_msg.unique_id = -2;
    // handTrajectoryPublisher.publish(hand_msg);

    ros::Duration(1).sleep();

    //  while (ros::ok())
    // {

  //  stateMach(state);
    ros::spinOnce();
    loop_rate.sleep();
    //}

    return 0;
}

ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos)
{

    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::OneDoFJointTrajectoryRosMessage t;

    for (int i=0;i<7;i++)
    {
        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        t.trajectory_points.push_back(p);
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return msg;
}

ihmc_msgs::PelvisHeightTrajectoryRosMessage controlPelvisHeight(float height)
{
    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::PelvisHeightTrajectoryRosMessage msg;

    p.position = height;
    p.velocity = 0.5;
    p.time = 0.0;

    msg.trajectory_points.push_back(p);
    msg.unique_id = 13;

    return msg;
}


