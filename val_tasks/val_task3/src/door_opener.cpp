#include "val_task3/door_opener.h"


doorOpener::doorOpener(ros::NodeHandle nh):nh_(nh),armTraj_(nh_),gripper_(nh_), task3_(nh_){

}

doorOpener::~doorOpener(){

}

void doorOpener::openDoor(){

    task3_.beforDoorOpenPose();


}
