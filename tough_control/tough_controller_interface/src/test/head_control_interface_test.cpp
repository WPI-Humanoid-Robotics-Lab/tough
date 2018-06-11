//Author: Syon Khosla
//Date (of last edit): April 20, 2018
//NOT COMPLETED
//VINAYAK'S JOB, NOT MINE

#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/head_control_interface.h>

TEST(headControl, headOrientation)
{
    const double pi = 3.1415926535897;

    ros::NodeHandle nh;
    HeadControlInterface head(nh);

    float roll = 10;
    float pitch = 10;
    float yaw = 30;
    float time = 2.0f;

    tf::Quaternion q1(head.getHeadOrientation().x, head.getHeadOrientation().y, head.getHeadOrientation().z, head.getHeadOrientation().w); //Quaternion with values before move

    tf::Matrix3x3 m1(q1); //Matrix with quaternion values
    ros::spinOnce();
    double rollActual1, pitchActual1, yawActual1;
    m1.getRPY(rollActual1, pitchActual1, yawActual1); //Getting values from robot's head
    std::cout << rollActual1 * 360 / (2 * pi) << " " << pitchActual1 * 360 / (2 * pi) << " " << yawActual1 * 360 / (2 * pi) << " " << std::endl;

    head.moveHead(roll, pitch, yaw, time); //Head moving
    ros::Duration(4).sleep();

    ROS_INFO("Motion finished!");
    ros::spinOnce();

    tf::Quaternion q(head.getHeadOrientation().x, head.getHeadOrientation().y, head.getHeadOrientation().z, head.getHeadOrientation().w);
    tf::Matrix3x3 m(q); //Matrix with new quaternion values (after move)
    ros::spinOnce();
    double rollActual, pitchActual, yawActual;
    m.getRPY(rollActual, pitchActual, yawActual);
    std::cout << "Roll: " << rollActual << std::endl;
    //std::cout << rollActual * 360 / (2 * pi) << " " << pitchActual * 360 / (2 * pi) << " " << yawActual * 360 / (2 * pi) << " " << std::endl;

    //std::cout << degree2rad((rollActual1 - rollActual)) << " " << roll << std::endl;

    ASSERT_LT(std::abs(roll - (rollActual)) , 0.05);
    //ASSERT_LT(std::abs(pitch - pitchActual), 0.05);
    //ASSERT_LT(std::abs(yaw - yawActual), 0.05);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_pelvis");

    return RUN_ALL_TESTS();
}

