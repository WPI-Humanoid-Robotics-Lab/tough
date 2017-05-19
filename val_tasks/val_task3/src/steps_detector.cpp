#include "val_task3/steps_detector.h"

steps_detector::steps_detector(ros::NodeHandle & nh) : nh_(nh)
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
    pcl_pub_.publish(cloud);
}
