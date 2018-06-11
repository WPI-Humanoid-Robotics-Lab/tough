//Author: Syon Khosla
//Date (of last edit): April 20, 2018
//NOT COMPLETED
//VINAYAK'S JOB, NOT MINE

#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/chest_control_interface.h>

double degree2rad(double val){
    return val*360/(2*M_PI);
}

TEST(chestcontrol, chestorientation)
{
        const double pi = 3.1415926535897;

        ros::NodeHandle nh;
        ChestControlInterface chest(nh);

        float roll = 10;
        float pitch = 10;
        float yaw = 30;
        float time = 2.0f;

        tf::Quaternion q1(chest.getChestOrientation().x, chest.getChestOrientation().y, chest.getChestOrientation().z, chest.getChestOrientation().w); //Quaternion with values before move

        tf::Matrix3x3 m1(q1); //Matrix with quaternion values
        ros::spinOnce();
        double rollActual1, pitchActual1, yawActual1;
        m1.getRPY(rollActual1, pitchActual1, yawActual1); //Getting
        std::cout << rollActual1 * 360 / (2 * pi) << " " << pitchActual1 * 360 / (2 * pi) << " " << yawActual1 * 360 / (2 * pi) << " " << std::endl;

        chest.controlChest(roll, pitch, yaw, time);
        ros::Duration(4).sleep();

        ROS_INFO("Motion finished!");
        ros::spinOnce();

        tf::Quaternion q(chest.getChestOrientation().x, chest.getChestOrientation().y, chest.getChestOrientation().z, chest.getChestOrientation().w);
        tf::Matrix3x3 m(q);
        ros::spinOnce();
        double rollActual, pitchActual, yawActual;
        m.getRPY(rollActual, pitchActual, yawActual);
        std::cout << rollActual * 360 / (2 * pi) << " " << pitchActual * 360 / (2 * pi) << " " << yawActual * 360 / (2 * pi) << " " << std::endl;

        std::cout << degree2rad((rollActual1 - rollActual)) << " " << roll << std::endl;

        ASSERT_LT(std::abs(roll - (std::abs(rollActual1 - rollActual) * 360 / (2 * pi))), 0.05);
        //ASSERT_LT(std::abs(pitch - pitchActual), 0.05);
        //ASSERT_LT(std::abs(yaw - yawActual), 0.05);
}

int main(int argc, char **argv)
{
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "test_pelvis");

        return RUN_ALL_TESTS();
}
