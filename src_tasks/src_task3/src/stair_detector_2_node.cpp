//
// Created by will on 5/31/17.
//
#include <ros/ros.h>
#include "src_task3/stair_detector_2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stairs_detector_2", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    stair_detector_2 detector(nh);
    std::vector<geometry_msgs::Pose> detections;

    while(!detector.getDetections(detections)) {
        ROS_INFO_STREAM_THROTTLE(5, "Stairs detections: " << detections.size());
        ros::spinOnce();
    }
    return 0;

}
