#include "val_task3/steps_detector.h"

steps_detector::steps_detector(ros::NodeHandle& nh) : nh_(nh)
{
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &steps_detector::stepsCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_steps/cloud2", 1);
}

steps_detector::~steps_detector()
{
    pcl_sub_.shutdown();
}

void steps_detector::stepsCB(const sensor_msgs::PointCloud2::Ptr& input)
{
    if (input->data.empty()){
        return;
    }
    //++detection_tries_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //sensor_msgs::PointCloud2 output;
    pcl::fromROSMsg(*input, *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);
    ROS_INFO_STREAM(cloud_normals->points[0] << std::endl);
//    Eigen::Vector4f plane_parameters;
//    float curvature;
//    pcl::computePointNormal(*cloud, plane_parameters, curvature);
//    pcl::CropBox<pcl::PointXYZ> box_filter;
//    ROS_INFO_STREAM(plane_parameters << std::endl);
    pcl::toROSMsg(*cloud_normals, *input);
    pcl_pub_.publish(*input);

}
