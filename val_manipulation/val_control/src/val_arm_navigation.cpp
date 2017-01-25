#include<val_control/val_arm_navigation_.h>
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

void armTrajectory::buttonPressArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    float BUTTON_PRESS_PREPARE [] ={1.57, -0.1, -1.6, 1.55, 0.0, 0.0, 0.0};
    float BUTTON_PRESS_ACT [] ={1.57, -0.3, -1.6, 1.3, 0.0, 0.0, 0.0};

    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    arm_traj = appendTrajectoryPoint(arm_traj, 2, BUTTON_PRESS_PREPARE);
    arm_traj = appendTrajectoryPoint(arm_traj, 2, BUTTON_PRESS_ACT);

    armTrajectoryPublisher.publish(arm_traj);
}

void armTrajectory::walkPoseArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();


    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    arm_traj = appendTrajectoryPoint(arm_traj, 3, armTrajectory::WALK_POSE);

    armTrajectoryPublisher.publish(arm_traj);
}

void armTrajectory::zeroPoseArm(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    arm_traj = appendTrajectoryPoint(arm_traj, 2, armTrajectory::ZERO_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveArm(armSide side, std::vector<float> arm_pose, float time){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();


    arm_traj.joint_trajectory_messages.resize(7);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
    arm_traj = appendTrajectoryPoint(arm_traj, time/arm_pose.size(), arm_pose);

    armTrajectoryPublisher.publish(arm_traj);
}

void moveArmMessage(ihmc_msgs::ArmTrajectoryRosMessage& msg){

    this->armTrajectoryPublisher.publish(msg);
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
}


//    ihmc_msgs::HandDesiredConfigurationRosMessage hand_msg;
//    hand_msg.robot_side = hand_msg.RIGHT;
//    hand_msg.hand_desired_configuration = hand_msg.CLOSE;
//    hand_msg.unique_id = -2;
//    handTrajectoryPublisher.publish(hand_msg);








