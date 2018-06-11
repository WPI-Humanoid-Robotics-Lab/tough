//Author: Syon Khosla
//Date (of last edit): April 20, 2018
//COMPLETED

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
        ros::spinOnce();
        std::cout << "hello" << std::endl;
        ros::spinOnce();
    }
    virtual void SetUp()
    {
        ros::spinOnce();
        positions.resize(0);
        ROS_INFO("Motion start");
        ros::spinOnce();
    }

    virtual void TearDown()
    {
        ros::spinOnce();
        positions.resize(0);
        ros::Duration(2).sleep();
        ROS_INFO("Motion Finished");
        ros::spinOnce();
    }
};

TEST_F(armsTestFixture, controlArmsJoint)
{
    //Joint 0
    ros::spinOnce();

    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 0, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[0] = -0.5;
    std::vector<float> positions2;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);

    //Joint 1
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 1, 1.25);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[1] = 1.25;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);


    //Joint 2
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 2, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[2] = -0.5;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);


    //Joint 3
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 3, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[3] = -0.5;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);


    //Joint 4
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 4, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[4] = -0.5;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);

    //Joint 5
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 5, -0.5);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[5] = -0.5;

    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);

    //Joint 6
    ros::spinOnce();
    a.getJointAngles(LEFT, positions);

    ros::spinOnce();
    a.moveArmJoint(LEFT, 6, -0.3);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    positions[6] = -0.3;

    //Checking values, making sure they moved to correct position.
    ros::spinOnce();
    a.getJointAngles(LEFT, positions2);
    for(int i = 0; i < positions.size(); i++){
    //    std::cout<<"Syon Test" << positions[i] << " " << positions2[i] << std::endl;
        ASSERT_LE(std::abs(positions[i] - positions2[i]), 0.05);
    }
}

int main(int argc, char **argv)
{
    //Starting google test framework, then ros node
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_pelvis");

    return RUN_ALL_TESTS();
}
