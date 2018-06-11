//Author: Syon Khosla
//Date (of last edit): April 18th, 2018
//COMPLETED

#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    //Initialize controllers
    GripperControlInterface gripper(nh);
    ArmControlInterface arm(nh);

    //Reach arm out towards the wheel

    arm.moveArmInTaskSpace();
    //Grip wheel

    //Rotate wheel

    //Unhand wheel

    //Retract arm
}


