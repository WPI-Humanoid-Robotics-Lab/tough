#include "src_task2/button_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findButtonDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    geometry_msgs::Point ButtonLoc;
    tough_perception::MultisenseImage* ms_sensor = new tough_perception::MultisenseImage(nh);
    ButtonDetector b1(nh, ms_sensor);

    //while(ros::ok())
    while (!foundButton && numIterations < 20)
    {
        foundButton = b1.findButtons(ButtonLoc);
        ROS_INFO(foundButton ? "***** Button detected" : "xxxxx button not detected");
        numIterations++;
    }

}

