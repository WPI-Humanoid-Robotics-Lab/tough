//Author: Syon Khosla
//Date (of last edit): April 20, 2018
//COMPLETED

#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/pelvis_control_interface.h>

class pelvisTestFixture: public testing::Test {
public:
    ros::NodeHandle nh;
    PelvisControlInterface p;
    float current_height;
    pelvisTestFixture():p(nh)
    {
        std::cout << "Constructor being called" << std::endl;
        current_height = p.getPelvisHeight();

    }
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
        p.controlPelvisHeight(current_height);
        ros::Duration(2).sleep();
        ROS_INFO("Motion Finished");
    }
};

TEST_F(pelvisTestFixture, controlPelvisHeight)
{
    //Towards the top of Pelvis range test
    float height = 1.0;
    p.controlPelvisHeight(height);
    ros::Duration(3).sleep();
    std::cout << "Height = " << p.getPelvisHeight() << std::endl;
    ASSERT_LT(std::abs(p.getPelvisHeight() - height), 0.05);

    //Towards the bottom of pelvis range test
    float height2 = 0.75;
    p.controlPelvisHeight(height2);
    ros::Duration(2).sleep();
    std::cout << "Height = " << p.getPelvisHeight() << std::endl;
    ASSERT_LT(std::abs(p.getPelvisHeight() - height2), 0.05);
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_pelvis");

	return RUN_ALL_TESTS();
}
