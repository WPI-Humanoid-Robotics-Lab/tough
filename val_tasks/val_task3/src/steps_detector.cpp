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
    pcl::fromROSMsg(*input, *cloud_);
    geometry_msgs::Point stairLoc;
    uint numSideBarsDetected;
    sd_.findStair(stairLoc, numSideBarsDetected);
    planeSegmentation(sd_.coefficients(), stairLoc);
}

void steps_detector::planeSegmentation(const std::vector<double>& coefficients, const geometry_msgs::Point& stairLoc)
{
    int count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr o (new pcl::PointCloud<pcl::PointXYZ>);
    o->points.clear();
    for (size_t i = 0; i < cloud_->size(); i++)
    {
        double threshold = coefficients[0] * cloud_->points[i].x + coefficients[1] * cloud_->points[i].y + coefficients[2] * cloud_->points[i].z - coefficients[3];
        if (threshold < 0.005 && threshold > -0.0){
            o->points.push_back(cloud_->points[i]);
            count++;
        }
    }
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(o);
    pass.setFilterFieldName ("x");
    pcl::PointCloud<pcl::PointXYZ>::Ptr o1 (new pcl::PointCloud<pcl::PointXYZ>);
    pass.setFilterLimits (3.68824, 6.6);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*o1);

//    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.003);

//    seg.setInputCloud (o1);
//    seg.segment (*inliers, *coefficients1);

//    ROS_INFO_STREAM("Coefficients: " << inliers->indices.size() << std::endl);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr o2 (new pcl::PointCloud<pcl::PointXYZ>);
//    o2->points.clear();
//    for (size_t i = 0; i < inliers->indices.size(); i++)
//    {
//        o2->points.push_back(o1->points[inliers->indices[i]]);
//    }

    double temp_point_x = o1->points[0].x;
    std::sort(o1->points.begin(), o1->points.end(), less_than_key());
    geometry_msgs::Point dir = sd_.dirVector();
    //double temp_point_z = o1->points[2].z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr o2 (new pcl::PointCloud<pcl::PointXYZ>);
    double cos_angle_temp = 0;
    for (size_t i = 1; i < o1->size(); i++)
    {
        double norm_cloud = std::sqrt((std::pow(o1->points[i].x, 2) + std::pow(o1->points[i].y, 2) + std::pow(o1->points[i].z, 2)));
        double cos_angle = ((o1->points[i].x - stairLoc.x) * dir.x + (o1->points[i].y - stairLoc.y) * dir.y + (o1->points[i].z - stairLoc.z) * dir.z)*100 / double(norm_cloud);
        ROS_INFO_STREAM("angle : " << cos_angle << std::endl);
        if (std::abs(o1->points[i].x - temp_point_x) > 0.03) //&& std::abs(o2->points[i].z - temp_point_z) < 0.01 )
        {
            o2->points.push_back(o1->points[i-1]);
        }
        temp_point_x = o1->points[i].x;

        //temp_point_z = o2->points[i-1].z;

    }
    //ROS_INFO_STREAM("Count" << o3->points.size() << std::endl);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*o1, output);
    output.header.frame_id="world";
    pcl_pub_.publish(output);
}
