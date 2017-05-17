#include "val_task3/stair_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findStairDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundStair = false;
    geometry_msgs::Point StairLoc;
    stair_detector s1(nh);
    uint numSideBarsDetected;
    //while (!foundStair && numIterations < 20)
    while(ros::ok())
    {
        foundStair = s1.findStair(StairLoc, numSideBarsDetected);
        //ROS_INFO(foundStair ? "***** Stair detected" : "xxxxx Stair not detected");
        numIterations++;
    }

}


