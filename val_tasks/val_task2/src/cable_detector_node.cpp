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
    geometry_msgs::Pose CablePose;
    CableDetector c1(nh);

    //while (!c1.findCable(CableLoc)){
    while(ros::ok()){
        //c1.findCable(CableLoc);
        c1.findCable(CablePose);
            //while (!foundCable && numIterations < 20)
        ROS_INFO("Cable location x:%f y:%f z:%f", CableLoc.x, CableLoc.y, CableLoc.z);
        ROS_INFO("Cable location x:%f y:%f z:%f", CablePose.position.x, CablePose.position.y, CablePose.position.z);
        ros::spinOnce();
    }




}

