#include "val_task3/leak_detector.h"


leakDetector::leakDetector(ros::NodeHandle nh):
    nh_(nh)
{
    leak_sb_ = nh_.subscribe("/task3/checkpoint5/leak", 10, &leakDetector::leakMsgCB, this);
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

    int points_in_vertical_line = (ver_high_limit - ver_low_limit)/VERTICAL_WIDTH;
    int points_in_horizontal_line = (horz_left_top.x - horz_right_bottom.x)/HORIZONTAL_WIDTH;

    geometry_msgs::Point point;
    point.x = horz_left_top.x;
    point.y = horz_left_top.y;
    point.z = ver_high_limit;

    for (int i=0; i<points_in_horizontal_line; i++)
    {
        for (int j=0; j<points_in_vertical_line; j++)
        {
            point.z = (i%2 == 0) ? point.z-VERTICAL_WIDTH : point.z+VERTICAL_WIDTH;
            way_points.push_back(point);
        }
        point.y += HORIZONTAL_WIDTH;
    }
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

