#include <tough_controller_interface/arm_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ArmControlInterface armInt(nh);

    armJointData a;
    armJointData b;
    armJointData c; 
    armJointData d;
    armJointData e;
    //we are moving the left arm (no reason why, it just didn’t matter which one we chose)
    a.side = LEFT;
    b.side = LEFT;
    c.side = LEFT;
    d.side = LEFT;
    e.side = LEFT;
    
//vectors of angles of joints at each position
    //Consider using while loop to alter vector and attac
    std::vector<float> vect1 {0, 0, 0, 0, 0, 0, 0};
    std::vector<float> vect2 {0.174, 0.15, 0.1, 0, 0.2, 0.10, 0.15};
    std::vector<float> vect3 {0.174, 0.30, 0.2, 0, 0.3, 0.20, 0.25};
    std::vector<float> vect4 {0.174, 0.35, 0.3, 0, 0.40, 0.28, 0.30};
    std::vector<float> vect5 {0.174, 0.35, 0.4, 0, 0.5, 0.28, 0.34};

//assigning each “ArmJointData” arm_pose to a position (that is described by a vector of angles for each joint)
    a.arm_pose =  vect1;
    b.arm_pose = vect2;
    c.arm_pose = vect3;
    d.arm_pose = vect4;
    e.arm_pose = vect5;

//this is the time taken for each position to be reached
    a.time = 2;
    b.time = 4;
    c.time = 6;
    d.time = 8;
    e.time = 10;

    std::vector <armJointData> data {a, b, c, d, e};
     
    //move the pelvis and arms (ALMOST simultaneously, just for demonstration) - if      simultaneous motion is wanted, you must use whole body message to control certain aspects of the robot simultaneously
    armInt.moveArmJoints(data);

    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}

