#include "val_task3/steps_detector.h"

steps_detector::steps_detector(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
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
    if (input->data.empty())
        return;
    //add mutex
    mtx_.lock();
    pcl::fromROSMsg(*input, *cloud_);
    mtx_.unlock();
}

void steps_detector::getStepsPosition(const std::vector<double>& coefficients, const geometry_msgs::Point& dir_vector, const geometry_msgs::Point& stair_loc)
{
    coefficients_ = coefficients;
    dirVector_= dir_vector;
    stairLoc_ = stair_loc;

    ROS_INFO_STREAM("1a" << std::endl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    out_cloud1->clear();
    out_cloud2->clear();
    output_cloud->clear();
    ROS_INFO_STREAM("1b" << std::endl);
    mtx_.lock();
    input_cloud = cloud_;
    mtx_.unlock();
    //copy cloud_ in a new variable using mutex
ROS_INFO_STREAM("1c" << std::endl);
    planeSegmentation(input_cloud, out_cloud1);
    ROS_INFO_STREAM("1d" << std::endl);
    zAxisSegmentation(out_cloud1, out_cloud2);
    ROS_INFO_STREAM("1e" << std::endl);
    stepCentroids(out_cloud2, output_cloud);
    ROS_INFO_STREAM("1f" << std::endl);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*output_cloud, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);

}

void steps_detector::planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    int count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_output (new pcl::PointCloud<pcl::PointXYZ>);
    temp_output->points.clear();
    for (size_t i = 0; i < input->size(); i++)
    {
        double threshold = coefficients_[0] * input->points[i].x + coefficients_[1] * input->points[i].y + coefficients_[2] * input->points[i].z - coefficients_[3];
        //double threshold = 0.244626 * input->points[i].x + -2.54532 * input->points[i].y + 0 * input->points[i].z - 3.85578;
        if (threshold < 0.01 && threshold > -0.01){
            temp_output->points.push_back(input->points[i]);
            count++;
        }
    }
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(temp_output);
    pass.setFilterFieldName ("x");
    //pass.setFilterLimits (3.62829, 6.7);
    pass.setFilterLimits (stairLoc_.x, stairLoc_.x + 3.1);
    ROS_INFO_STREAM(stairLoc_.x << std::endl);
    ROS_INFO_STREAM(temp_output->size() << std::endl);
    pass.filter (*output);
       ROS_INFO_STREAM(output->size() << std::endl);
}

void steps_detector::zAxisSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    std::sort(input->points.begin(), input->points.end(), comparePoints);
        ROS_INFO_STREAM("1da" << std::endl);
    pcl::PointXYZ temp_point = input->points[0];
ROS_INFO_STREAM("1dab" << std::endl);
    for (size_t i = 1; i < input->size(); i++)
    {
        double dot_product = ((input->points[i].x - temp_point.x) * 0 + (input->points[i].y - temp_point.y) * 0 + (input->points[i].z - temp_point.z) * 1);

        if (std::abs(dot_product) > 0.01)
            output->points.push_back(input->points[i]);
        ROS_INFO_STREAM("1db" << std::endl);
        temp_point = input->points[i];
    }
    ROS_INFO_STREAM("1dc" << std::endl);
}

void steps_detector::stepCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    std::sort(input->points.begin(), input->points.end(), comparePoints);
    pcl::PointXYZ temp_point1 = {0 ,0 ,0};
    pcl::PointXYZ temp_point = input->points[0];
    size_t j = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    temp_cloud->clear();
    for (size_t i = 1 ; i < input->size(); i++, j++)
    {
        double dot_product = ((input->points[i].x - temp_point.x) * dirVector_.x + (input->points[i].y - temp_point.y) * dirVector_.y + (input->points[i].z - temp_point.z) * dirVector_.z);

        if (std::abs(dot_product) > 0.04 && j > 5)
        {
            temp_point1.x /= double(j);
            temp_point1.y /= double(j);
            temp_point1.z /= double(j);
            temp_cloud->points.push_back(temp_point1);
            temp_point1 = {0 ,0 ,0};
            j = 0;
        }
        temp_point1.x += input->points[i].x;
        temp_point1.y += input->points[i].y;
        temp_point1.z += input->points[i].z;
        temp_point = input->points[i];
    }
    temp_point1.x /= double(j);
    temp_point1.y /= double(j);
    temp_point1.z /= double(j);
    temp_cloud->points.push_back(temp_point1);

    pcl::PointXYZ temp_point2;
    temp_point2 = {0, 0, 0};
    for (size_t i = 0; i < temp_cloud->size() - 1; i++)
    {
        temp_point2.x = (temp_cloud->points[i].x + temp_cloud->points[i+1].x)/2.0;
        temp_point2.y = (temp_cloud->points[i].y + temp_cloud->points[i+1].y)/2.0;
        temp_point2.z = (temp_cloud->points[i].z + temp_cloud->points[i+1].z)/2.0;

        output->points.push_back(temp_point2);
    }

//    for (size_t i = 0; i < o4->size() - 1; i++)
//    {
//        ROS_INFO_STREAM("x : " << o4->points[i+1].x - o4->points[i].x << std::endl);
//    }
}

