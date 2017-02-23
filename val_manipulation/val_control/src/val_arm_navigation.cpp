#include<val_control/val_arm_navigation.h>
#include<stdlib.h>

//add default pose for both arms. the values of joints are different.
armTrajectory::armTrajectory(ros::NodeHandle nh):nh_(nh),
    ZERO_POSE{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    DEFAULT_POSE{-0.2f, 1.2f, 0.7222f, 1.5101f, 0.0f, 0.0f, 0.0f},
    NUM_ARM_JOINTS(7){

    armTrajectoryPublisher = nh_.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory", 1,true);
    handTrajectoryPublisher = nh_.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/valkyrie/control/hand_desired_configuration", 1,true);
}

armTrajectory::~armTrajectory(){

}


void armTrajectory::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos)
{

    for (int i=0;i<NUM_ARM_JOINTS;i++)
    {
        ihmc_msgs::TrajectoryPoint1DRosMessage p;
        ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
        t.trajectory_points.clear();

        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        p.unique_id = armTrajectory::arm_id;
        t.trajectory_points.push_back(p);
        t.unique_id = armTrajectory::arm_id;
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return;
}

void armTrajectory::moveToDefaultPose(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    appendTrajectoryPoint(arm_traj, 3, DEFAULT_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveToZeroPose(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    appendTrajectoryPoint(arm_traj, 2, ZERO_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveArmJoints(const armSide side, const std::vector<std::vector<float>> &arm_pose,const float time){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();


    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
    for(auto i=arm_pose.begin(); i != arm_pose.end(); i++){
           if(i->size() != NUM_ARM_JOINTS)
           ROS_ERROR("Check number of trajectory points");
        appendTrajectoryPoint(arm_traj, time/arm_pose.size(), *i);
    }

    armTrajectoryPublisher.publish(arm_traj);
}


void armTrajectory::moveArmJoints(std::vector<armJointData> &arm_data){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_r;
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_l;

    arm_traj_r.joint_trajectory_messages.clear();
    arm_traj_r.joint_trajectory_messages.resize(NUM_ARM_JOINTS);

    armTrajectory::arm_id--;
    arm_traj_r.unique_id = armTrajectory::arm_id;
    armTrajectory::arm_id--;
    arm_traj_l.joint_trajectory_messages.clear();
    arm_traj_l.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj_l.unique_id = armTrajectory::arm_id;

    for(std::vector<armJointData>::iterator i=arm_data.begin(); i != arm_data.end(); i++){

        if(i->arm_pose.size() != NUM_ARM_JOINTS){
            ROS_INFO("Check number of trajectory points");
            return;
        }

        if(i->side == RIGHT){
            arm_traj_r.robot_side = i->side;
            appendTrajectoryPoint(arm_traj_r, i->time, i->arm_pose);
        }

        else {
            arm_traj_l.robot_side = i->side;
            appendTrajectoryPoint(arm_traj_l, i->time, i->arm_pose);
        }

    }

    armTrajectoryPublisher.publish(arm_traj_r);
    ros::Duration(0.02).sleep();
    armTrajectoryPublisher.publish(arm_traj_l);
}


void armTrajectory::moveArmMessage(const ihmc_msgs::ArmTrajectoryRosMessage &msg){
    this->armTrajectoryPublisher.publish(msg);
    armTrajectory::arm_id--;

}
int armTrajectory::getnumArmJoints() const
{
    return NUM_ARM_JOINTS;
}
