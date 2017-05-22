#include "val_task3/steps_detector.h"

steps_detector::steps_detector(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>), sd_(nh)
{
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &steps_detector::stepsCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_steps/cloud2", 1, true);

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

    //sensor_msgs::PointCloud2 output;
    pcl::fromROSMsg(*input, *cloud_);
    geometry_msgs::Point stairLoc;
    uint numSideBarsDetected;
    sd_.findStair(stairLoc, numSideBarsDetected);
    planeSegmentation(sd_.coefficients());
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);

//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);
    //ROS_INFO_STREAM(cloud_->points[0].x << std::endl);


}

void steps_detector::planeSegmentation(const std::vector<double>& coefficients)
{
    int count = 0;
    pcl::PointCloud<pcl::PointXYZ> o;
    o.points.clear();
    for (size_t i = 0; i < cloud_->size(); i++)
    {
        double threshold = coefficients[0] * cloud_->points[i].x + coefficients[1] * cloud_->points[i].y + coefficients[2] * cloud_->points[i].z - coefficients[3];
        if (threshold < 0.1 && threshold > -0.1){
            o.points.push_back(cloud_->points[i]);
            count++;
        }
    }
    ROS_INFO_STREAM("Count" << count << std::endl);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(o, output);
    output.header.frame_id="world";
    pcl_pub_.publish(output);
}
