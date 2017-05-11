#include <val_task2/cable_detector.h>

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findCableDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundCable = false;
    geometry_msgs::Point CableLoc;
    cable_detector c1(nh);

    //while(ros::ok())
    while (!foundCable && numIterations < 20)
    {
        foundCable = c1.findCable(CableLoc);
        ROS_INFO(foundCable ? "***** Cable detected" : "xxxxx Cable not detected");
        numIterations++;
    }

}

