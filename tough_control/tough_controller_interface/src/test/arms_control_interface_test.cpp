//Syon Khosla
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
    ros::spinOnce();
    //std::cout << "In test" << std::endl;
    //positions = {0, 0, 0, 0, 0, 0, 0};
    //caling getJointPositions():
    a.getJointAngles(LEFT, positions);
    //std::cout << "Is it getting the joint angles?" << isItWorking << std::endl;
    //This is just a debug loop
    //for(int i = 0; i < positions.size(); i++){
    //    std::cout << "Initial values: " << positions[i] << std::endl;
    //}
    //caling getJointPositions():
    ros::spinOnce();
    a.moveArmJoint(LEFT, 0, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[0] = -0.5;
    std::vector<float> positions2;
    //caling getJointPositions():
    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
    //for(int i = 0; i < positions.size(); i++){
    //    std::cout<<"Syon Test" << positions[i] << " " << positions2[i] << std::endl;
    //    ASSERT_LE(std::abs(positions[i] - positions2[i]), 0.05);
    //}
}

TEST_F(armsTestFixture, controlArmsJoint2)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 1, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[1] = -0.5;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
}

TEST_F(armsTestFixture, controlArmsJoint3)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 2, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[2] = -0.5;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
}

TEST_F(armsTestFixture, controlArmsJoint4)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 3, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[3] = -0.5;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
}

TEST_F(armsTestFixture, controlArmsJoint5)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 4, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[4] = -0.5;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
}

TEST_F(armsTestFixture, controlArmsJoint6)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 5, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[5] = -0.5;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
}

TEST_F(armsTestFixture, controlArmsJoint7)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 6, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[6] = -0.3;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
}

TEST_F(armsTestFixture, nudgeArm)
{
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 1, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[1] = -0.5;

    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
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
