#include "val_task2/button_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findButtonDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    geometry_msgs::Point ButtonLoc;
    button_detector b1(nh);

    //while(ros::ok())
    while (!foundButton && numIterations < 20)
    {
        foundButton = b1.findButtons(ButtonLoc);
        //ROS_INFO(foundButton ? "***** Button detected" : "xxxxx button not detected");
        numIterations++;
    }

}

