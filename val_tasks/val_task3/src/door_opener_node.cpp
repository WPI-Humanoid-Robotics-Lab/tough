#include "val_task3/door_opener.h"

int main(int argc, char** argv){


    /** This code should acceot the centre of valve from perception
     *  It's hard coded presently
     */

    ros::init(argc, argv, "door_opener_node");
    ros::NodeHandle nh;
    doorOpener doorOpen(nh);
    geometry_msgs::Pose valveCenter;

    valveCenter.position.x = std::atof(argv[1]);
    valveCenter.position.y = std::atof(argv[2]);
    valveCenter.position.z = std::atof(argv[3]);
    ROS_INFO("Starting door open task");

    doorOpen.openDoor(valveCenter);

    ros::spin();

    return 0;
}
