#ifndef WALKWAY_FILTER_H
#define WALKWAY_FILTER_H

#define LOWER_THRESHOLD -0.05f
#define UPPER_THRESHOLD  0.09f
#include <ros/ros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

class WalkwayFilter{
public:
    WalkwayFilter(ros::NodeHandle &n);
    ~WalkwayFilter();
    void generateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr msg);
    void subtractPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices::Ptr outliers);
private:
    ros::Publisher mapPub_;
    ros::Publisher pointcloudPub_;
    ros::Subscriber pointcloudSub_;
    ros::NodeHandle nh_;
};

#endif // WALKWAY_FILTER_H

