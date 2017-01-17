#include<val_manipulation/val_arm_navigation_.h>
#include<stdlib.h>
#include <stdio.h>


armTrajectory::armTrajectory(ros::NodeHandle nh):nh_(nh){

    armTrajectoryPublisher = nh_.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory", 1,true);
    handTrajectoryPublisher = nh_.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/valkyrie/control/hand_desired_configuration", 1,true);

}

armTrajectory::~armTrajectory(){

}


ihmc_msgs::ArmTrajectoryRosMessage armTrajectory::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos)
{

    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
    t.trajectory_points.clear();

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

void armTrajectory::buttonPressPrepareArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_r;
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_l;
    arm_traj_r.joint_trajectory_messages.clear();
    arm_traj_l.joint_trajectory_messages.clear();

    float BUTTON_PRESS_PREPARE_R [] = {0.0, 0.25, 0.2, 1.1, 0.0, 0.0, 0.0};
    float BUTTON_PRESS_PREPARE_L [] = {0.0, -0.25, 0.2, -1.1, 0.0, 0.0, 0.0};

    arm_traj_r.joint_trajectory_messages.resize(7);
    arm_traj_r.robot_side = RIGHT;
    arm_traj_r.unique_id = -11;

    arm_traj_l.joint_trajectory_messages.resize(7);
    arm_traj_l.robot_side = LEFT;
    arm_traj_l.unique_id = -21;

    arm_traj_r = appendTrajectoryPoint(arm_traj_r, 2, BUTTON_PRESS_PREPARE_R);
    arm_traj_l = appendTrajectoryPoint(arm_traj_l, 2, BUTTON_PRESS_PREPARE_L);


    armTrajectoryPublisher.publish(arm_traj_r);
    ros::Duration(0.01).sleep();
    armTrajectoryPublisher.publish(arm_traj_l);
}

void armTrajectory::buttonPressArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_r;
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_l;
    arm_traj_r.joint_trajectory_messages.clear();
    arm_traj_l.joint_trajectory_messages.clear();

    float BUTTON_PRESS_ACT_R [] ={1.57, -0.3, -1.6, 1.3, 0.0, 0.0, 0.0};
    float BUTTON_PRESS_ACT_L [] ={-1.57, 0.3, 1.6, -1.3, 0.0, 0.0, 0.0};

    arm_traj_r.joint_trajectory_messages.resize(7);
    arm_traj_r.robot_side = RIGHT;
    arm_traj_r.unique_id = -1;
    arm_traj_r = appendTrajectoryPoint(arm_traj_r, 1, BUTTON_PRESS_ACT_R);

    arm_traj_l.joint_trajectory_messages.resize(7);
    arm_traj_l.robot_side = LEFT;
    arm_traj_l.unique_id = -2;
    arm_traj_l = appendTrajectoryPoint(arm_traj_l, 1, BUTTON_PRESS_ACT_L);

    armTrajectoryPublisher.publish(arm_traj_r);
    armTrajectoryPublisher.publish(arm_traj_l);
}

void armTrajectory::walkPoseArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_r;
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_l;
    arm_traj_r.joint_trajectory_messages.clear();
    arm_traj_l.joint_trajectory_messages.clear();

    float RETRACT_TO_ACTUAL_R [] ={0.0, 1.2, 1.2, 0.4, 0.0, 0.0, 0.0}; //{-0.2, 1.2, 0.7222, 1.5101, 0.0, 0.0, 0.0};
    float RETRACT_TO_ACTUAL_L [] ={0.0, -1.2, 1.2, -0.4, 0.0, 0.0, 0.0};

    arm_traj_r.joint_trajectory_messages.resize(7);
    arm_traj_r.robot_side = RIGHT;
    arm_traj_r.unique_id = -5;
    arm_traj_r = appendTrajectoryPoint(arm_traj_r, 1, RETRACT_TO_ACTUAL_R);

    arm_traj_l.joint_trajectory_messages.resize(7);
    arm_traj_l.robot_side = LEFT;
    arm_traj_l.unique_id = -6;
    arm_traj_l = appendTrajectoryPoint(arm_traj_l, 1, RETRACT_TO_ACTUAL_L);


    armTrajectoryPublisher.publish(arm_traj_r);
    ros::Duration(0.01).sleep();
    armTrajectoryPublisher.publish(arm_traj_l);
}

void armTrajectory::zeroPoseArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    float ZERO_POSE [] ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = side;
    arm_traj.unique_id = -1;

    arm_traj = appendTrajectoryPoint(arm_traj, 2, ZERO_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}



//    ihmc_msgs::HandDesiredConfigurationRosMessage hand_msg;
//    hand_msg.robot_side = hand_msg.RIGHT;
//    hand_msg.hand_desired_configuration = hand_msg.CLOSE;
//    hand_msg.unique_id = -2;
//    handTrajectoryPublisher.publish(hand_msg);








