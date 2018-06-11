//Author: Syon Khosla
//Date (of last edit): June 5th, 2018
//COMPLETED

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

void populateMarkerWithoutPose(visualization_msgs::Marker &marker){
    static int i = 0;
    marker.header.frame_id = "pelvis";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose = pose;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

}

void populatePosition(geometry_msgs::Point &point, const double &x, const double &y, const double &z ){
    point.x = x;
    point.y = y;
    point.z = z;
}

void populateOrientation(geometry_msgs::Quaternion &orientation, const double &x, const double &y, const double &z, const double &w){
    orientation.w = w;
    orientation.x = x;
    orientation.y = y;
    orientation.z = z;
}

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_marker_node");
    ros::NodeHandle node_handle;

    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );

    visualization_msgs::Marker marker1;
    populatePosition(marker1.pose.position, 1,1,1);
    populateOrientation(marker1.pose.orientation, 0, 0, 0, 1);
    populateMarkerWithoutPose(marker1);

    visualization_msgs::Marker marker2;
    populatePosition(marker2.pose.position, 0,1,1);
    populateOrientation(marker2.pose.orientation, 0, 0, 0, 1);
    populateMarkerWithoutPose(marker2);

    //visualization_msgs::Marker marker3;
    //populatePosition(marker3.pose.position, 0,1,0);
    //populateOrientation(marker3.pose.orientation, 0, 0, 0, 1);
    //populateMarkerWithoutPose(marker3);

    //visualization_msgs::Marker marker4;
    //populatePosition(marker4.pose.position, 0,0,1);
    //populateOrientation(marker4.pose.orientation, 0, 0, 0, 1);
    //populateMarkerWithoutPose(marker4);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker1);
    ma.markers.push_back(marker2);
    //ma.markers.push_back(marker3);
    //ma.markers.push_back(marker4);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        vis_pub.publish(ma);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

