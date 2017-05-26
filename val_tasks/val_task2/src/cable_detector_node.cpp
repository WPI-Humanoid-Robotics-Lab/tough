#include <val_task2/cable_detector.h>

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findCableDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundCable = false;
    bool foundStandPos = false;
    geometry_msgs::Point CableLoc;
    geometry_msgs::Point StandLoc;
    CableDetector c1(nh);
    foundCable = c1.findCable(CableLoc);
    while(ros::ok())
    //while (!foundCable && numIterations < 20)
    {

        foundStandPos = c1.getStandPosition(StandLoc);
        //ROS_INFO(foundCable ? "***** Cable detected" : "xxxxx Cable not detected");
        ros::spinOnce();
        numIterations++;
    }

}

