/*
 * Author : Vinayak Jagtap (vvjagtap@wpi.edu)
 *
 * This node is a manual bug fix for a bug in octomap server.
 * The orientation of 2D map provided by octomap server returns nan for yaw of origin.
 * This node publishes 0 yaw for origin and publishes the new map on /map topic
 *
 */
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <pcl/range_image/range_image_planar.h>
#include <val_common/val_common_names.h>
#include <val_filters/walkway_filter.h>

WalkwayFilter::WalkwayFilter(ros::NodeHandle &n):nh_(n)
{
    pointcloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("walkway_filtered_points2",1);
    pointcloudSub_ = nh_.subscribe("assembled_cloud2", 1,  &WalkwayFilter::generateMap, this);
}

WalkwayFilter::~WalkwayFilter()
{
    mapPub_.shutdown();
    pointcloudSub_.shutdown();
}

void WalkwayFilter::generateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(cloud->empty())
        return;

    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
    outliers->header = cloud->header;

    for (size_t i = 0; i< cloud->size(); ++i){
        if( cloud->at(i).z > LOWER_THRESHOLD && cloud->at(i).z < UPPER_THRESHOLD){
             outliers->indices.insert(outliers->indices.end(),i);
        }
    }
    subtractPointClouds(cloud, outliers);
    pointcloudPub_.publish(cloud);
}

void WalkwayFilter::subtractPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices::Ptr outliers){
    pcl::ExtractIndices<pcl::PointXYZ> extract ;
    extract.setInputCloud(full_cloud);
    extract.setIndices(outliers);
    extract.setNegative (true);
    extract.filter (*full_cloud);
    return;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "walkway_filter");
    ROS_INFO("Starting walkway filter node");
    ros::NodeHandle n;
    WalkwayFilter m(n);
    ros::spin();
    return 0;
}
