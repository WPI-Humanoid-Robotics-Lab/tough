#include "val_task3/door_opener.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "door_opener_node");
    ros::NodeHandle nh;
    doorOpener doorOpen(nh);

    ROS_INFO("Starting door open task");

    doorOpen.openDoor();

    ros::spin();

    return 0;
}
