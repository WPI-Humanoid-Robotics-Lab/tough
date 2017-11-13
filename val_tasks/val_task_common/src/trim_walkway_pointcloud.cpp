#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <tough_common/val_common_names.h>
#include <perception_common/perception_common_names.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


void trimPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
int i=0;
ros::Subscriber pointcloudSub;
ros::Publisher pointcloudPub;


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "trim_walkway");
    ros::NodeHandle nh;

    ROS_INFO("A");
        pointcloudSub = nh.subscribe("/field/walkway", 1,  trimPoints);
    pointcloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("trimmed_walkway",1,true);

    ROS_INFO("B");
    ros::spin();
    return 0;
}

void trimPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) //const sensor_msgs::PointCloud2 cloud_in
{
    ROS_INFO("C");
    if(cloud->empty())
        return;
    std::cout<<i++;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
            cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    ROS_INFO("D");
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
              << inliers->indices.size () << " inliers." << std::endl;

    ROS_INFO("E");
    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->points.size () << " data points." << std::endl;

    ROS_INFO("F");
    // Create a Concave Hull representation of the projected inliers
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);
    cloud_hull->header = cloud->header;
    cloud_hull->header.frame_id = VAL_COMMON_NAMES::WORLD_TF; //VAL_COMMON_NAMES::WORLD_TF

    std::cerr << "Concave hull has: " << cloud_hull->points.size ()
              << " data points." << std::endl;
    ROS_INFO("G");
    std::cout<<cloud_hull->header.frame_id;
    pointcloudPub.publish(cloud_hull);

}
