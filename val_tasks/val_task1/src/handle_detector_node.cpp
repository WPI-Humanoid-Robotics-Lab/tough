#include "val_task1/handle_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findHandleDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    std::vector<geometry_msgs::Point> handleLocs;
    HandleDetector h1(nh);

    while (numIterations <200)
    {
        foundButton = h1.findHandles(handleLocs);
        ROS_INFO(foundButton ? "***** handles detected" : "xxxxx handles not detected");
        numIterations++;
    }

}

