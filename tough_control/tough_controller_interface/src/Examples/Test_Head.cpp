#include <tough_controller_interface/head_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    HeadControlInterface h(nh);

    float roll = 30;
    float pitch = 15;
    float yaw = 10;
    float time = 3;
    
    h.moveHead(roll, pitch, yaw, time);
    
    ros::Duration(2).sleep(); 
    ROS_INFO("Motion finished");

    return 0;
}
