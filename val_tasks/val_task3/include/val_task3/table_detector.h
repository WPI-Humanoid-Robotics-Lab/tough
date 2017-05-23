#ifndef TABLE_DETECTOR_H
#define TABLE_DETECTOR_H

#include <geometry_msgs/Point.h>

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

class table_detector
{
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher points_pub_;
    ros::Subscriber pcl_sub_;

    src_perception::MultisensePointCloud point_cloud_listener_;

    std::vector<geometry_msgs::PoseStamped> vantage_poses_;

public:

    typedef pcl::PointXYZ          Point;
    typedef pcl::PointCloud<table_detector::Point> PointCloud;

    table_detector(ros::NodeHandle nh);

    void cloudCB(const table_detector::PointCloud::ConstPtr &cloud);
    bool findTable(std::vector<geometry_msgs::PoseStamped> &detected_pt);

    ~table_detector();

};
#endif // TABLE_DETECTOR_H
