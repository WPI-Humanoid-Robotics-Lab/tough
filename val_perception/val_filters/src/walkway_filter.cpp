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
#include <perception_common/perception_common_names.h>

WalkwayFilter::WalkwayFilter(ros::NodeHandle &n):nh_(n)
{
    pointcloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("walkway_filtered_points2",1);
    pointcloudSub_ = nh_.subscribe(PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_FILTERED_CLOUD_TOPIC2, 1,  &WalkwayFilter::generateMap, this);
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

    float foot_height = getCurrentFootPose();
    std::cout<<  foot_height << std::endl;

    for (size_t i = 0; i< cloud->size(); ++i){
        if( cloud->at(i).z < foot_height - FOOT_GROUND_THRESHOLD
                && cloud->at(i).z > foot_height - FOOT_GROUND_THRESHOLD - (GROUND_THRESHOLD)){
            outliers->indices.insert(outliers->indices.end(),i);
        }
    }
    subtractPointClouds(cloud,outliers);
    pointcloudPub_.publish(cloud);
}

double WalkwayFilter::getCurrentFootPose(void)
{
    double height_foot;

    tf::StampedTransform transformStamped;
    tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::L_FOOT_TF, ros::Time(0),transformStamped);
    height_foot = transformStamped.getOrigin().getZ();

    tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::R_FOOT_TF, ros::Time(0),transformStamped);

    height_foot = height_foot > transformStamped.getOrigin().getZ() ? transformStamped.getOrigin().getZ() : height_foot;

    return height_foot;
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
