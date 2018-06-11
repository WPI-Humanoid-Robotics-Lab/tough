//Author: Syon Khosla
//Date (of last edit): April 18th, 2018
//COMPLETED

#include <tough_controller_interface/arm_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    //Declaring arm interface and initialize with NodeHandle as parameter to constructor
    ArmControlInterface armInt(nh);

    //Declaring armJointData objects, as you need a vector of these for the arm to move through a certain path
    armJointData a;
    a.side = LEFT;
    std::vector<float> vect1 {0, 0, 0, 0, 0, 0, 0};
    a.arm_pose =  vect1;
    a.time = 2;

    armJointData b;
    b.side = LEFT;
    std::vector<float> vect2 {0.174, 0.15, 0.1, 0, 0.2, 0.10, 0.15};
    b.arm_pose = vect2;
    b.time = 4;

    armJointData c; 
    c.side = LEFT;
    std::vector<float> vect3 {0.174, 0.30, 0.2, 0, 0.3, 0.20, 0.25};
    c.arm_pose = vect3;
    c.time = 6;

    armJointData d;
    d.side = LEFT;
    std::vector<float> vect4 {0.174, 0.35, 0.3, 0, 0.40, 0.28, 0.30};
    d.arm_pose = vect4;
    d.time = 8;

    armJointData e;
    e.side = LEFT;
    std::vector<float> vect5 {0.174, 0.35, 0.4, 0, 0.5, 0.28, 0.34};
    e.arm_pose = vect5;
    e.time = 10;

    //armJointData vector that defines path that arm will travel through to get to final position
    std::vector <armJointData> data {a, b, c, d, e};
     
    //Move the left arm through the defined path by us in the vectors above!
    armInt.moveArmJoints(data);

    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}

