//
// Created by will on 5/31/17.
//
#include <ros/ros.h>
#include "val_task2/solar_panel_detection_2.h"

int main(int argc, char** argv)
{
    // TODO LIST:
    // 1. Pass a pose describing the rover pose to the constructor
    // 2. In prefilterCloud, crop to the approximately rover trailer walls and floor
    // 3. Do region growing segmentation to identify walls and floor
    // 4. Crop to the inside of the detected walls and floor
    // 5. Find the pose of the box -- ICP? SampleConsensusPrerejective? Just the plane of the side of the box?
    ros::init(argc, argv, "solar_panel_detector_2");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool found = false;

    // Fake rover pose -- assumes the robot is standing in front of the rover
    geometry_msgs::PoseStamped rover_pose;
    rover_pose.header.frame_id = "leftFoot";
    rover_pose.pose.position.x = 0.40;
    rover_pose.pose.position.y = -0.05;
    rover_pose.pose.orientation.w = 1;

    solar_panel_detector_2 detector(nh, rover_pose);
    //while (!found && numIterations < 20)
    while(ros::ok())
    {
//        found = detector.findSolarPanel();

        numIterations++;
        ros::spinOnce();
    }

}
