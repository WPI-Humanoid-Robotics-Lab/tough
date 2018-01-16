#include <tough_controller_interface/gripper_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "test_gripper_control");
    ros::NodeHandle nh;
    GripperControlInterface gripcont(nh);

    ros::Publisher log_pub = nh.advertise<std_msgs::String>(TOUGH_COMMON_NAMES::LOG_TOPIC, 10);
    const auto log_msg = [&log_pub](const std::string &str) {
        std_msgs::String msg;
        msg.data = ros::this_node::getName() + ": " + str;
        log_pub.publish(msg);
        ROS_INFO("%s", msg.data.c_str());
    };

    // wait a reasonable amount of time for the subscriber to connect
    ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
    while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until) {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
    }

    std::vector<double> leftGrip,rightGrip;
    if(argc == 2 ){
        if (argv[1][0] == '1'){
            log_msg("opening both grippers");
            gripcont.openGripper(LEFT);
            gripcont.openGripper(RIGHT);
        }
        else {
            log_msg("closing both grippers");
            gripcont.closeGripper(LEFT);
            gripcont.closeGripper(RIGHT);
        }

    }
    else if(argc == 3){
        RobotSide side = (RobotSide)std::atoi(argv[1]);
        GRIPPER_STATE state = (GRIPPER_STATE)std::atoi(argv[2]);

        std::string side_str = side == RobotSide::LEFT ? "left" : "right";
        std::string state_str = ((const char*[]) {"OPEN", "OPEN_THUMB_IN", "OPEN_THUMB_IN_APPROACH", "C LOSE",
                                                  "HANDLE_HOLD", "TIGHT_HOLD", "CUP"})[(int) state];

        log_msg("moving " + side_str + " gripper to " + state_str);
        gripcont.controlGripper(side, state);
    }
    else if(argc == 7)
    {
        if (argv[1][0] == '1'){
            // Right Selected
            rightGrip.clear();
            rightGrip.push_back(std::atof(argv[2]));
            rightGrip.push_back(std::atof(argv[3]));
            rightGrip.push_back(std::atof(argv[4]));
            rightGrip.push_back(std::atof(argv[5]));
            rightGrip.push_back(std::atof(argv[6]));
            log_msg("moving right gripper to given grip");
            gripcont.controlGripper(RIGHT,rightGrip);
        }
        else
        {
            // Left Selected
            leftGrip.clear();
            leftGrip.push_back(std::atof(argv[2]));
            leftGrip.push_back(std::atof(argv[3]));
            leftGrip.push_back(std::atof(argv[4]));
            leftGrip.push_back(std::atof(argv[5]));
            leftGrip.push_back(std::atof(argv[6]));
            log_msg("moving left gripper to given grip");
            gripcont.controlGripper(LEFT,leftGrip);
        }
    }
    else {
        log_msg("Usage: rosrun <node_name> 1 \n to open grippers \n running demo");
        std::vector<double> leftgripClose = {1.4, -0.55, 0.0, -0.9, -1.0};
        std::vector<double> rightgripClose = {1.4, 0.55, 0.0, 0.9, 1.0};

        gripcont.controlGripper(LEFT,leftgripClose);
        gripcont.controlGripper(RIGHT,rightgripClose);
    }
    ros::spinOnce();
    ros::Duration(2).sleep();

    log_msg("motion complete");
    return 0;
}
