#include <val_task2/cable_detector.h>

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findCableDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    geometry_msgs::Point CableLoc;
    cable_detector c1(nh);

    while(ros::ok())
    //while (!foundButton && numIterations < 20)
    {
        c1.findCable(CableLoc);
        //foundButton = b1.findButtons(ButtonLoc);
        //ROS_INFO(foundButton ? "***** Button detected" : "xxxxx button not detected");
        numIterations++;
    }

}

