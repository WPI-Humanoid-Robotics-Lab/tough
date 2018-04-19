#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
    //std::cout << pelvisInt.getPelvisHeight() << std::endl;
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");

    return 0;
}
