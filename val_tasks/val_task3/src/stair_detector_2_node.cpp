//
// Created by will on 5/31/17.
//
#include <ros/ros.h>
#include "val_task3/stair_detector_2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stairs_detector_2");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool found = false;

    stair_detector_2 detector(nh);
    //while (!found && numIterations < 20)
    while(ros::ok())
    {
        found = detector.findStair();

        numIterations++;
        ros::spinOnce();
    }

}
