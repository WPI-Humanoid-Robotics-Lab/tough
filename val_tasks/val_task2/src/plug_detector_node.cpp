#include "val_task2/plug_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findPlugDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundPlug = false;
    geometry_msgs::Point PlugLoc;

    src_perception::MultisenseImage* ms_sensor = new src_perception::MultisenseImage(nh);
    SocketDetector p1(nh,ms_sensor);

    //while(ros::ok())
    while (!foundPlug && numIterations < 20)
    {
        foundPlug = p1.findPlug(PlugLoc);
        ROS_INFO(foundPlug ? "***** Plug detected" : "xxxxx Plug not detected");
        numIterations++;
    }

}
