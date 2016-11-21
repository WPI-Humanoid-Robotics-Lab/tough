#include<val_manipulation/val_arm_navigation_.h>
#include<stdlib.h>

ihmc_msgs::OneDoFJointTrajectoryRosMessage appendTrajectoryPoint(float time, float* pos);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_navigation");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher armTrajectoryPublisher = nh.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("arm_trajectory_publisher", 1,true);
    ros::Publisher handTrajectoryPublisher = nh.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("hand_desired_configuration", 1,true);

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;

    float ZERO_VECTOR [] ={0.0, -1.0, 2.0, 1.0, 0.0, 0.0, 0.0};
    float ELBOW_BENT_UP [] ={0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0};
    float BUTTON_PRESS [] ={0.0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0};

    arm_traj.robot_side = ihmc_msgs::ArmTrajectoryRosMessage::RIGHT;
    arm_traj.joint_trajectory_messages.push_back( appendTrajectoryPoint(2, ZERO_VECTOR));
    arm_traj.joint_trajectory_messages.push_back( appendTrajectoryPoint(3, ELBOW_BENT_UP));
    arm_traj.joint_trajectory_messages.push_back( appendTrajectoryPoint(4, ZERO_VECTOR));
    arm_traj.joint_trajectory_messages.push_back( appendTrajectoryPoint(5, BUTTON_PRESS));

    armTrajectoryPublisher.publish(arm_traj);

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}

ihmc_msgs::OneDoFJointTrajectoryRosMessage appendTrajectoryPoint(float time, float* pos)
{

    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::OneDoFJointTrajectoryRosMessage t;

    for (int i=1;i<7;i++)
    {
        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        t.trajectory_points.push_back(p);
    }

    return t;
}
