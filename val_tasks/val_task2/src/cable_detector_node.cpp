#include <val_task2/cable_detector.h>

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findCableDetector");
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Rate loop(1);
    pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);
    int numIterations = 0;
    bool foundStandPos = false;
    geometry_msgs::Point CableLoc;
    geometry_msgs::Pose StandLoc;
    CableDetector c1(nh);

    //while (!c1.findCable(CableLoc)){
    while(ros::ok()){
        c1.findCable(CableLoc);
            //while (!foundCable && numIterations < 20)
        //ROS_INFO("Cable detection failed. retrying");
        ros::spinOnce();
    }




}

