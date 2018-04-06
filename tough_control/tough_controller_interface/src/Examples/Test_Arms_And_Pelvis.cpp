#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ArmControlInterface armInt(nh);
    PelvisControlInterface pelvInt(nh);
    
    float height = 1.05;

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
    std::vector<float> vect1 {0, 0, 0, 0, 0, 0, 0};
    std::vector<float> vect2 {10, 10, 10, 0, 10, 10, 10};
    std::vector<float> vect3 {10, 15, 20, 0, 15, 20, 25};
    std::vector<float> vect4 {10, 20, 20, 0, 23, 28, 25};
    std::vector<float> vect5 {10, 27, 20, 0, 23, 28, 25};

//assigning each “ArmJointData” arm_pose to a position (that is described by a vector of angles for each joint)
    a.arm_pose =  vect1;
    b.arm_pose = vect2;
    c.arm_pose = vect3;
    d.arm_pose = vect4;
    e.arm_pose = vect5;

//this is the time taken for each position to be reached
    a.time = 2;
    b.time = 2;
    c.time = 2;
    d.time = 2;
    e.time = 2;

    std::vector <armJointData> data {a, b, c, d, e};

    //move the pelvis and arms (ALMOST simultaneously, just for demonstration) - if      simultaneous motion is wanted, you must use whole body message to control certain aspects of the robot simultaneously
    armInt.moveArmJoints(data);
    ros::Duration(0.2).sleep();
    pelvInt.controlPelvisHeight(height);
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
