#include <tough_controller_interface/gripper_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    GripperControlInterface g(nh);

    g.controlGripper(LEFT, GRIPPER_STATE::CLOSE);

    ros::Duration(2).sleep();

    ROS_INFO("Motion finished");
    return 0;
}
