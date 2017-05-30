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

    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time::now();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.05;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    while (!c1.findCable(CableLoc)){
        ROS_INFO("Cable detection failed. retrying");
    }
    while(ros::ok())
    //while (!foundCable && numIterations < 20)
    {

        foundStandPos = c1.getStandPosition(StandLoc);
        marker.pose = StandLoc;
        pub.publish(marker);
        ROS_INFO("Robot Pose: x:%f y:%f z:%f", StandLoc.position.x, StandLoc.position.y, StandLoc.position.z);
        ros::spinOnce();
        loop.sleep();
        numIterations++;
    }

}

