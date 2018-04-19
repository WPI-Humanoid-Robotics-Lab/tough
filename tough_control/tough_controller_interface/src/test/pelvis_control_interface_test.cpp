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
    float height = 1.0;
    p.controlPelvisHeight(height);
    ros::Duration(2).sleep();
    ASSERT_LT(std::abs(p.getPelvisHeight() - height), 0.05);
}

TEST_F(pelvisTestFixture, controlPelvisHeight2)
{
    float height = 0.75;
    p.controlPelvisHeight(height);
    ros::Duration(2).sleep();
    ASSERT_LT(std::abs(p.getPelvisHeight() - height), 0.05);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_pelvis");
    ros::NodeHandle nh_;

//    PelvisControlInterface pelvInterface(nh_);

//    pelvInterface.controlPelvisHeight(1.0);

//    ros::Duration(2).sleep();

//    ROS_INFO("Motion finished!");
	return RUN_ALL_TESTS();
}
