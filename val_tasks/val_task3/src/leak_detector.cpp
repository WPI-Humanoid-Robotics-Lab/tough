#include "val_task3/leak_detector.h"


leakDetector::leakDetector(ros::NodeHandle nh):
    nh_(nh)
{
    leak_sb_ = nh_.subscribe("/task3/checkpoint5/leak", 10, &leakDetector::leakMsgCB, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "leak_search_points", 10, true);
}

leakDetector::~leakDetector()
{
    //shutdown subscribers
    leak_sb_.shutdown();
}

void leakDetector::leakMsgCB(const srcsim::Leak &leakmsg)
{
    leak_value_ = leakmsg.value;
}


void leakDetector::generateSearchWayPoints(geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom, float ver_low_limit, float ver_high_limit, std::vector<geometry_msgs::Point>& way_points)
{
    // generate way points with the dimension of the field of view of the tool

    float points_in_vertical_line = (ver_high_limit - ver_low_limit)/VERTICAL_WIDTH;
    float points_in_horizontal_line = (horz_right_bottom.x - horz_left_top.x)/HORIZONTAL_WIDTH;

    geometry_msgs::Point point;
    point.x = horz_left_top.x;
    point.y = horz_left_top.y;
    point.z = ver_high_limit;

    int z=0;
    for (float i=0; i<points_in_horizontal_line; i+=VERTICAL_WIDTH, z++)
    {
        for (float j=0; j<points_in_vertical_line; j+=HORIZONTAL_WIDTH)
        {
            point.z = (z%2 == 0) ? point.z-VERTICAL_WIDTH : point.z+VERTICAL_WIDTH;
            way_points.push_back(point);
        }
        point.y += HORIZONTAL_WIDTH;
        way_points.push_back(point);
    }

    ROS_INFO("%d search points generated", way_points.size());
    // visulaize points
    visulatise3DPoints(way_points);
}

void leakDetector::findLeak (geometry_msgs::Point& leak_point)
{
    // get the way points

    // plan the path

}

// helper functions
double leakDetector::getLeakValue() const
{
    return leak_value_;
}

void leakDetector::setLeakValue(double leak_value)
{
    leak_value_ = leak_value;
}

void leakDetector::visulatise3DPoints(std::vector<geometry_msgs::Point> &points)
{

    visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();

    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "leak";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);

    for (int i=0; i<points.size();i++)
    {
        marker.id = i;
        marker.pose.position = points[i];
        marker_array.markers.push_back(marker);
    }

    marker_pub_.publish(marker_array);
    ros::Duration(0.2).sleep();
}

