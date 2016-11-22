#include<val_manipulation/val_arm_navigation_.h>
#include<stdlib.h>
#include <stdio.h>

ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_navigation");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher armTrajectoryPublisher = nh.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory", 1,true);
    ros::Publisher handTrajectoryPublisher = nh.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/valkyrie/control/hand_desired_configuration", 1,true);

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;

    float ZERO_VECTOR [] ={0.0, -1.0, 2.0, 1.0, 0.0, 0.0, 0.0};
    float ELBOW_BENT_UP [] ={0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0};
    float BUTTON_PRESS [] ={0.0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0};


    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = arm_traj.RIGHT;
    arm_traj.unique_id = -1;
    arm_traj = appendTrajectoryPoint(arm_traj,2, ZERO_VECTOR);
    arm_traj = appendTrajectoryPoint(arm_traj, 3, ELBOW_BENT_UP);
    arm_traj = appendTrajectoryPoint (arm_traj, 4, ZERO_VECTOR);
    arm_traj = appendTrajectoryPoint(arm_traj, 5, BUTTON_PRESS);

    //armTrajectoryPublisher.publish(arm_traj);

    ihmc_msgs::HandDesiredConfigurationRosMessage hand_msg;
    hand_msg.robot_side = hand_msg.RIGHT;
    hand_msg.hand_desired_configuration = hand_msg.PINCH_GRIP;
    hand_msg.unique_id = -2;
    handTrajectoryPublisher.publish(hand_msg);

    ros::Duration(1).sleep();

    //  while (ros::ok())
    // {

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
