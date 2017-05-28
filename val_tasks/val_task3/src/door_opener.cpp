#include "val_task3/door_opener.h"


doorOpener::doorOpener(ros::NodeHandle nh)
    :nh_(nh),armTraj_(nh_),gripper_(nh_), task3_(nh_), walker_(nh){

}

doorOpener::~doorOpener(){

}

void doorOpener::openDoor(){

    std::vector<float> x_offset={0.0,-0.1};
    std::vector<float> y_offset={0.2,0.0};
    walker_.walkLocalPreComputedSteps(x_offset,y_offset,LEFT);

    ros::Duration(2.0).sleep();
    task3_.beforDoorOpenPose();
    ros::Duration(0.5).sleep();

    // Walk straight few small steps
    x_offset = {0.2,0.2,0.2,0.1,0};
    y_offset = {0.0,0.0,0.0,0.0,-0.2};
    walker_.walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);
    ros::Duration(2.0).sleep();

}
