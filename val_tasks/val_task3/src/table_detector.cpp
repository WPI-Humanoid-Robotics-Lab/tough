#include "val_task3/table_detector.h"
#include <visualization_msgs/Marker.h>
#include "val_common/val_common_names.h"
#include <pcl/common/centroid.h>
#include <thread>

#define DISABLE_DRAWINGS true
//#define DISABLE_TRACKBAR true

table_detector::table_detector(ros::NodeHandle nh) : nh_(nh)
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_table",1);
}

table_detector::~table_detector()
{

}
