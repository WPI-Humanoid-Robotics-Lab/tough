#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/head_control_interface.h>

class headTestFixture: public testing::Test {
public:
    ros::NodeHandle nh;
    HeadControlInterface h;
    headTestFixture():h(nh)
    {
    }
    virtual void SetUp()
    {
        ros::spinOnce();
    }

    virtual void TearDown()
    {
        ros::spinOnce();
    }
};

TEST_F(headTestFixture, testHead)
{
    ros::spinOnce();
    std::cout << "Value: " << h.getHeadOrientation().w << std::endl;
    ASSERT_LT(std::abs(h.getHeadOrientation().w - 0.95107), 0.05);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_head");
    ros::NodeHandle nh_;

    HeadControlInterface headInterface(nh_);

    //Moves the head for Roll: 30 degrees, Pitch: 15 degrees, and Yaw: 10 degrees
    headInterface.moveHead(30.0, 15.0, 10.0);

    //Sleep for movement to occur
    ros::Duration(2).sleep();

    //Message for when motion is done
    ROS_INFO("Motion finished!");

    return RUN_ALL_TESTS();
}

