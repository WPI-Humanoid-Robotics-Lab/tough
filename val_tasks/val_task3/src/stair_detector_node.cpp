#include "val_task3/stair_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findStairDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundStair = false;
    geometry_msgs::Point StairLoc;
    stair_detector s1(nh);
    //while (!foundStair && numIterations < 20)
    while(ros::ok())
    {
        s1.findStair(StairLoc);
        //ROS_INFO(foundStair ? "***** Stair detected" : "xxxxx Stair not detected");
        numIterations++;
    }

}


