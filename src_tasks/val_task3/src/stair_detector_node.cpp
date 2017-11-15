#include "val_task3/stair_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"Stair_Detector_Node");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool found_stairs = false;

    geometry_msgs::Point StairLocation;
    StairDetector stair(nh);
    uint numSideBarsDetected;
    //while (!foundStair && numIterations < 20)
    while(ros::ok())
    {
        found_stairs = stair.findStair(StairLocation, numSideBarsDetected);
        ROS_INFO(found_stairs ? "***** Stair detected" : "xxxxx Stair not detected");
        numIterations++;
        if(numIterations>10)
        {
            break;
        }
    }

}


