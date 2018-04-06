#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/arm_control_interface.h>

class armsTestFixture: public testing::Test {
public:
    ros::NodeHandle nh;
    ArmControlInterface a;
    std::vector<float> positions;
    armsTestFixture():a(nh)
    {
        std::cout << "hello" << std::endl;
    }
    virtual void SetUp()
    {
        positions.resize(0);
    }

    virtual void TearDown()
    {
        positions.resize(0);
    }
};

TEST_F(armsTestFixture, controlArmsJoint1)
{
    std::cout << "In test" << std::endl;
    //positions = {0, 0, 0, 0, 0, 0, 0};
    //caling getJointPositions():
    a.getJointAngles(LEFT, positions);
    //This is just a debug loop
    for(int i = 0; i < positions.size(); i++){
        std::cout << "Initial values: " << positions[i] << std::endl;
    }
    //caling getJointPositions():
    a.moveArmJoint(LEFT, 0, 1.0);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[0] = 1.0;
    std::vector<float> positions2;
    //caling getJointPositions():
    a.getJointAngles(LEFT, positions2);
    for(int i = 0; i < positions.size(); i++){
        std::cout<<"Syon Test" << positions[i] << " " << positions2[i] << std::endl;
        ASSERT_LE(std::abs(positions[i] - positions2[i]), 0.05);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_pelvis");
    ros::NodeHandle nh_;

    ros::Duration(2).sleep();

    ROS_INFO("Motion finished!");
    return RUN_ALL_TESTS();
}
