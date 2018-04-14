#include <tough_controller_interface/arm_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of ArmControlInterface - used for actually altering the different aspects of the arms of the robot
    ArmControlInterface armInt(nh);

    // change the Right arm position to ZeroPose. This is a non-blocking call.
    armInt.moveToZeroPose(RIGHT);

    // wait for the robot to move
    ros::Duration(2).sleep();
	
    //Moving Left arm to ZeroPose.
    armInt.moveToZeroPose(LEFT);

    //Waiting for movement to finish
    ros::Duration(2).sleep();

    //Closes the hand of the left arm
    armInt.closeHand(LEFT);
    ros::Duration(2).sleep();

    //Closes right arm's hand
    armInt.closeHand(RIGHT);
    ros::Duration(2).sleep();

    //Publish brief message
    ROS_INFO("Motion finished");

    return 0;
}

//End of program
