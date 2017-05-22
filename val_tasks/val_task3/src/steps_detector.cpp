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

//bool operator ()(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
//{
//    return ((std::pow(p1.x, 2) + std::pow(p1.y, 2) + std::pow(p1.z, 2)) < (std::pow(p2.x, 2) + std::pow(p2.y, 2) + std::pow(p2.z, 2)));
//}

void steps_detector::stepsCB(const sensor_msgs::PointCloud2::Ptr& input)
{
    if (input->data.empty()){
        return;
    }
    pcl::fromROSMsg(*input, *cloud_);
    geometry_msgs::Point stairLoc;
    std::vector<double> coefficients;
    uint numSideBarsDetected;
    sd_.findStair(stairLoc, numSideBarsDetected);
    coefficients = sd_.coefficients();
    ROS_INFO_STREAM("coefficients : " << coefficients[0] << "\t" << coefficients[1] << "\t"<< coefficients[2] << "\t"<< coefficients[3] << "\t" << std::endl);
    ROS_INFO_STREAM("dirVector : " << sd_.dirVector() << std::endl);
    planeSegmentation(coefficients, stairLoc);
}

void steps_detector::planeSegmentation(const std::vector<double>& coefficients, const geometry_msgs::Point& stairLoc)
{
    int count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr o (new pcl::PointCloud<pcl::PointXYZ>);
    o->points.clear();
    for (size_t i = 0; i < cloud_->size(); i++)
    {
        //double threshold = coefficients[0] * cloud_->points[i].x + coefficients[1] * cloud_->points[i].y + coefficients[2] * cloud_->points[i].z - coefficients[3];
        double threshold = 0.244626 * cloud_->points[i].x + -2.54532 * cloud_->points[i].y + 0 * cloud_->points[i].z - 3.85578;
        if (threshold < 0.01 && threshold > -0.01){
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
    o1->points.clear();
    pass.filter (*o1);

//    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
//    pcl::SACSegmentationFromNormals<pcl::PointXYZ> seg;
//    // Optional
//    //seg.setOptimizeCoefficients (true);
//    // Mandatory

//    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
//    seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
//    seg.setEpsAngle(0.501799);
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
//    // 0.244626	-2.54532	0	3.85578
//    //ROS_INFO_STREAM("coefficients : " << coefficients[0] << "\t" << coefficients[1] << "\t"<< coefficients[2] << "\t"<< coefficients[3] << "\t" << std::endl);
//    ROS_INFO_STREAM("coefficients : " << *coefficients1 << std::endl);


    std::sort(o1->points.begin(), o1->points.end(), less_than_key());
    geometry_msgs::Point dir = sd_.dirVector();
    pcl::PointXYZ temp_point = o1->points[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr o2 (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 1; i < o1->size(); i++)
    {
        double dot_product = ((o1->points[i].x - temp_point.x) * 0 + (o1->points[i].y - temp_point.y) * 0 + (o1->points[i].z - temp_point.z) * 1);
        //double cos_angle = ((o1->points[i].x - stairLoc.x) * dir.x + (o1->points[i].y - stairLoc.y) * dir.y + (o1->points[i].z - stairLoc.z) * dir.z)*100 / double(norm_cloud);
        ROS_INFO_STREAM("dot_product : " << dot_product << std::endl);
        if (std::abs(dot_product) > 0.01) //&& std::abs(o2->points[i].z - temp_point_z) < 0.01 )
        {
            o2->points.push_back(o1->points[i]);
        }
        temp_point = o1->points[i];
    }

    std::sort(o2->points.begin(), o2->points.end(), less_than_key());
    //geometry_msgs::Point dir = sd_.dirVector();
    temp_point = o2->points[0];
    pcl::PointCloud<pcl::PointXYZ>::Ptr o3 (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 1; i < o2->size(); i++)
    {
        double dot_product = ((o2->points[i].x - temp_point.x) * 1 + (o2->points[i].y - temp_point.y) * 0 + (o2->points[i].z - temp_point.z) * 0);
        //double cos_angle = ((o1->points[i].x - stairLoc.x) * dir.x + (o1->points[i].y - stairLoc.y) * dir.y + (o1->points[i].z - stairLoc.z) * dir.z)*100 / double(norm_cloud);
        ROS_INFO_STREAM("dot_product : " << dot_product << std::endl);
        if (std::abs(dot_product) > 0.04) //&& std::abs(o2->points[i].z - temp_point_z) < 0.01 )
        {
            o3->points.push_back(o2->points[i]);
            o3->points.push_back(o2->points[i-1]);
        }
        temp_point = o2->points[i];
    }

//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (o2);

//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//    int cloud_size = (int)o2->points.size();
//    ec.setClusterTolerance (0.03); // 2cm
//    ec.setMinClusterSize (3);
//    ec.setMaxClusterSize (cloud_size);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (o2);
//    ec.extract (cluster_indices);

//    ROS_INFO_STREAM("size : " << cluster_indices.size() << std::endl);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr o3 (new pcl::PointCloud<pcl::PointXYZ>);
//    o3->resize(o2->size());
//    size_t j = 0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, ++j)
//    {
//        Eigen::Vector4f cloudCentroid;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//            cloud_cluster->points.push_back (o2->points[*pit]);
//        pcl::compute3DCentroid(*cloud_cluster, cloudCentroid);
//        o3->points[j].x = cloudCentroid(0);
//        o3->points[j].y = cloudCentroid(1);
//        o3->points[j].z = cloudCentroid(2);
//    }
    //ROS_INFO_STREAM("Count" << o3->points.size() << std::endl);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*o3, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);
}
