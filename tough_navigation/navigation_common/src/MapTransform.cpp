#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

// This can be achieved in launch file using static frame publisher. We don't need this node.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_tf");
    ros::NodeHandle n;

    tf::TransformBroadcaster broadcaster;
    ros::Rate r(100);

    while(n.ok()){
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                        ros::Time::now(),"/world", "/map"));
        r.sleep();
    }

    return 0 ;
}
