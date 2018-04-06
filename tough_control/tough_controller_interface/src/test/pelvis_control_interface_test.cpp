#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/pelvis_control_interface.h>

class pelvisTestFixture: public testing::Test {
public:
    ros::NodeHandle nh;
    PelvisControlInterface p;
    pelvisTestFixture():p(nh)
    {
        std::cout << "hello" << std::endl;
    }
    virtual void SetUp()
    {

    }

    virtual void TearDown()
    {
        p.controlPelvisHeight(0.75);
        ros::Duration(2).sleep();
        ROS_INFO("Motion Finished");
    }
};

TEST_F(pelvisTestFixture, controlPelvisHeight)
{
    ASSERT_LT(std::abs(p.getPelvisHeight() - 1.0), 0.05);
}

TEST_F(pelvisTestFixture, controlPelvisHeight2)
{
    ASSERT_LT(std::abs(p.getPelvisHeight() - 0.75), 0.05);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_pelvis");
    ros::NodeHandle nh_;

    PelvisControlInterface pelvInterface(nh_);

    pelvInterface.controlPelvisHeight(1.0);

    ros::Duration(2).sleep();

    ROS_INFO("Motion finished!");
	return RUN_ALL_TESTS();
}
