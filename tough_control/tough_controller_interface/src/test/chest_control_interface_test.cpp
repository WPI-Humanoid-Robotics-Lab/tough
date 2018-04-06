#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/chest_control_interface.h>

void moveChest(ChestControlInterface c, float roll, float pitch, float yaw, float time)
{
        c.controlChest(roll, pitch, yaw, time);
        ros::Duration(2).sleep();

        ROS_INFO("Motion finished!");
}

TEST(chestcontrol, chestorientation)
{
        ros::NodeHandle nh;
        ChestControlInterface chest(nh);
        ASSERT_LT(std::abs(chest.getChestOrientation().w - 0.956), 0.05);
}

int main(int argc, char **argv)
{
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "test_pelvis");
        ros::NodeHandle nh;
        ChestControlInterface chestInterface(nh);
        moveChest(chestInterface, 10, 10, 30, 5.0f);
        ROS_INFO("Motion finished!");

        return RUN_ALL_TESTS();
}

