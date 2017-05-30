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

bool steps_detector::getStepsPosition(const std::vector<double>& coefficients, const geometry_msgs::Point& dir_vector, const geometry_msgs::Point& stair_loc, std::vector<pcl::PointXYZ>& step_points)
{
    bool steps_detected = false;

    if(!step_points.empty())
        step_points.clear();

    //get the coefficients, direction vector and the stair Location from the state machine
    coefficients_ = coefficients;
    dirVector_= dir_vector;
    stairLoc_ = stair_loc;

    if (cloud_->empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //copy cloud into local variable
    mtx_.lock();
    input_cloud = cloud_;
    mtx_.unlock();

    //Segment the plane and pass through filter to narrow the filter region to the stairs
    if (input_cloud->empty())
        return false;
    planeSegmentation(input_cloud, out_cloud1);

    //Remove points horizontal to the ground
    if (out_cloud1->empty())
        return false;
    zAxisSegmentation(out_cloud1, out_cloud2);

    //Find the step points by taking the midpoint of the centroids of the clusters obtained after the z-axis segmentation
    if (out_cloud2->empty())
        return false;
    stepCentroids(out_cloud2, output_cloud);

    if (!output_cloud->empty())
        steps_detected = true;

    for (size_t i = 0; i < output_cloud->points.size(); i++)
    {
       step_points.push_back(output_cloud->points[i]);
    }

    //publish the output cloud for visualization
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*output_cloud, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);

    //clear the points in the temporary clouds
    input_cloud->points.clear();
    out_cloud1->points.clear();
    out_cloud2->points.clear();
    output_cloud->points.clear();

    return steps_detected;
}

void steps_detector::planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    int count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_output (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < input->size(); i++)
    {
        double threshold = coefficients_[0] * input->points[i].x + coefficients_[1] * input->points[i].y + coefficients_[2] * input->points[i].z - coefficients_[3];
        //double threshold = 0.244626 * input->points[i].x + -2.54532 * input->points[i].y + 0 * input->points[i].z - 3.85578;
        if (threshold < 0.01 && threshold > -0.01){
            temp_output->points.push_back(input->points[i]);
            count++;
        }
    }
    if (temp_output->empty())
        return;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(temp_output);
    pass.setFilterFieldName ("x");
    //pass.setFilterLimits (3.62829, 6.7);
    pass.setFilterLimits (stairLoc_.x, stairLoc_.x + STAIR_LENGTH_FORWARD);
    pass.filter (*output);
    temp_output->points.clear();
}

void steps_detector::zAxisSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    std::sort(input->points.begin(), input->points.end(), comparePoints);
    pcl::PointXYZ temp_point = input->points[0];
    for (size_t i = 1; i < input->size(); i++)
    {
        double dot_product = ((input->points[i].x - temp_point.x) * 0 + (input->points[i].y - temp_point.y) * 0 + (input->points[i].z - temp_point.z) * 1);
        double norm_threshold = std::sqrt(std::pow(input->points[i-1].x - input->points[i].x, 2) + std::pow(input->points[i-1].z - input->points[i].z, 2));
                //ROS_INFO_STREAM("output :" << norm_threshold << std::endl);
        if (std::abs(dot_product) > 0.01 && norm_threshold > 0.03)
            output->points.push_back(input->points[i]);
        temp_point = input->points[i];
    }
}

void steps_detector::stepCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    std::sort(input->points.begin(), input->points.end(), comparePoints);
    pcl::PointXYZ temp_point1 = {0 ,0 ,0};
    pcl::PointXYZ temp_point = input->points[0];
    size_t j = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
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

    if (temp_cloud->empty())
        return;

    pcl::PointXYZ temp_point2;
    temp_point2 = {0, 0, 0};
    for (size_t i = 0; i < temp_cloud->size() - 1; i++)
    {
        temp_point2.x = (temp_cloud->points[i].x + temp_cloud->points[i+1].x)/2.0;
        temp_point2.y = (temp_cloud->points[i].y + temp_cloud->points[i+1].y)/2.0;
        temp_point2.z = (temp_cloud->points[i].z + temp_cloud->points[i+1].z)/2.0;

        output->points.push_back(temp_point2);
    }

    pcl::PointXYZ temp_point3;
    temp_point3.x = double(temp_point2.x + 0.2391737143);
    temp_point3.y = double(temp_point2.y);
    temp_point3.z = double(temp_point2.z + 0.2015475);

    output->points.push_back(temp_point3);

    for (size_t i = 0; i < output->size() - 1; i++)
    {
        //ROS_INFO_STREAM("y : " << output->points[i+1].y << std::endl);
        ROS_INFO_STREAM("x : " << output->points[i+1].x - output->points[i].x << std::endl);
    }
    temp_cloud->points.clear();
}

void steps_detector::visualize_point(geometry_msgs::Point point)
{
    static int id = 0;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = std::to_string(frameID_);
    marker.id = id++;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    markers_.markers.push_back(marker);
}
