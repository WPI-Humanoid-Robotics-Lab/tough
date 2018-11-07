#include <tough_controller_interface/gripper_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>


void demo_gripper(GripperControlInterface gripcont)
{
    std::vector<GripperControlInterface::GRIPPER_MODES> gripperModes{
                    GripperControlInterface::BASIC,
                    GripperControlInterface::PINCH,
                    GripperControlInterface::WIDE,
                    GripperControlInterface::SCISSOR};

    std::vector<RobotSide> gripperSide{RobotSide::LEFT};
    for(auto side:gripperSide)
    {

        for(auto mode:gripperModes)
        {
            ROS_INFO_STREAM("[mode] " << mode);
            gripcont.setMode(side,mode);
            ros::Duration(2).sleep();
            ROS_INFO("\t closing gripper");
            gripcont.closeGripper(side);
            ros::Duration(2).sleep();
            ROS_INFO("\t opening gripper");
            gripcont.openGripper(side);
            ros::Duration(2).sleep();

            ROS_INFO("\t opening Fingers");
            gripcont.closeFingers(side);
            ros::Duration(2).sleep();
            ROS_INFO("\t closing Fingers");
            gripcont.openFingers(side);
            ros::Duration(2).sleep();

            ROS_INFO("\t opening Thumb");
            gripcont.closeThumb(side);
            ros::Duration(2).sleep();
            ROS_INFO("\t closing Thumb");
            gripcont.openThumb(side);
            ros::Duration(2).sleep();
        }
        ros::Duration(2).sleep();
        ROS_INFO("Resetting gripper");
    //    gripcont.resetGripper(RobotSide::LEFT);
    //    ros::Duration(8).sleep();
        gripcont.closeGripper(side);
    }

}


int main(int argc, char **argv){

    ros::init(argc, argv, "test_gripper_control");
    ros::NodeHandle nh;
    GripperControlInterface gripcont(nh);
    std::cout << argc << std::endl;



    // wait a reasonable amount of time for the subscriber to connect
    ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
    while (ros::Time::now() < wait_until) {
        ros::WallDuration(0.1).sleep();
    }


    std::vector<double> leftGrip,rightGrip;
    if(argc == 2 ){
        if (argv[1][0] == '1'){
            ROS_INFO("opening both grippers");
            gripcont.openGripper(LEFT);
            gripcont.openGripper(RIGHT);
        }
        else {
            ROS_INFO("closing both grippers");
            gripcont.closeGripper(LEFT);
            gripcont.closeGripper(RIGHT);
        }

    }
    else if(argc == 3){
        RobotSide side = (RobotSide)std::atoi(argv[1]);
        int state = std::atoi(argv[2]);

        std::string side_str = side == RobotSide::LEFT ? "left" : "right";

        ROS_INFO_STREAM("moving " << side_str << " gripper to " << argv[2]);
        gripcont.controlGripper(side, state);
    }
    else {
        ROS_INFO("Usage: rosrun <node_name> 1 \n to open grippers \n running demo");
        demo_gripper(gripcont);

    }
    ros::spinOnce();
    ros::Duration(2).sleep();

    ROS_INFO("motion complete");
    return 0;
}
