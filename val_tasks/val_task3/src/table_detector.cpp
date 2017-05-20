// standard includes
#include <thread>
#include <algorithm>


// Library includes
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

// Local includes
#include "val_task3/table_detector.h"
#include <visualization_msgs/Marker.h>
#include "val_common/val_common_names.h"

#define DISABLE_DRAWINGS false
//#define DISABLE_TRACKBAR true

template<typename T>
std::tuple<T, T> largest_two(const T& a, const T& b, const T& c) {
    const T& largest = std::max({a, b, c});
    if (largest == a) return std::tuple<T, T>{a, std::max(b, c)};
    if (largest == b) return std::tuple<T, T>{b, std::max(a, c)};
    return std::tuple<T, T>{c, std::max(a, b)};
};

table_detector::table_detector(ros::NodeHandle nh) : nh_(nh), point_cloud_listener_(nh, "/leftFoot", "/left_camera_frame")
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_table",1);
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &table_detector::cloudCB, this);
    ROS_DEBUG("Table detector setup finished");
}

void table_detector::cloudCB(const table_detector::PointCloud::ConstPtr &cloud) {
    ROS_DEBUG_STREAM("Got point cloud of size " << cloud->size());
    // Run it through a z filter to approximately the height of the table
    pcl::PassThrough<table_detector::Point> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.7, 0.9); // TODO: Extract constants

    auto points_at_table_height = boost::make_shared<pcl::PointIndices>();
    pass.filter(points_at_table_height->indices);

    // Perform euclidean clustering to find candidates
    auto candidates = boost::make_shared<std::vector<pcl::PointIndices>>();
    pcl::EuclideanClusterExtraction<table_detector::Point> ec;
    ec.setClusterTolerance(0.04); // meters
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setInputCloud(cloud);
    ec.setIndices(points_at_table_height);
    ec.extract(*candidates);

    ROS_DEBUG_STREAM("Got " << candidates->size() << " candidate planes");

    // Use RANSAC on each candidate with PERPENDICULAR_PLANE w/r/t z-axis to get surface height
    pcl::SACSegmentation<table_detector::Point> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis({0.0, 0.0, 1.0});
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);

    // Get the table boundary using the smallest surrounding rectangle problem

    // Check each candidate for table legs:
    // Find points 20 +/- 2.5cm below the table surface and within the concave hull
    // Group points into "leg" and "not leg" based on proximity to rectangle corner
    // If there are enough "leg" points and not many "not leg" points, then approve the candidate

    // Check each remaining candidate for items on the table:
    // Chop the area 2cm-10cm above the table and within the table boundary
    // Euclidean segment it
    // If there are two segments, it is good

    visualization_msgs::MarkerArray markers;

    int marker_id = 0;
    for (const auto &candidate_indices : *candidates) {
        // seg takes shared_ptr to the indices, so make one that refers back to the shared_ptr to the vector of indices
        const pcl::PointIndices::ConstPtr cluster_ptr(candidates, &candidate_indices);
        seg.setIndices(cluster_ptr);

        auto inliers = pcl::PointIndices();
        auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
        seg.segment(inliers, *coefficients);

        // Get the table boundary by finding the oriented bounding box
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, inliers, centroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, inliers, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
        // Eigenvectors do not necessarily describe a right-handed coordinate system, this corrects that
        eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

        Eigen::Matrix4f table_frame_tf = Eigen::Matrix4f::Identity();
        table_frame_tf.block<3,3>(0,0) = eigenvectors.transpose();
        table_frame_tf.block<3,1>(0,3) = -1.f * (table_frame_tf.block<3,3>(0,0) * centroid.head<3>());
        table_detector::PointCloud table_frame_cloud;
        pcl::transformPointCloud(*cloud, inliers, table_frame_cloud, table_frame_tf);

        table_detector::Point min_pt, max_pt;
        pcl::getMinMax3D(table_frame_cloud, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
        // Final transform
        const Eigen::Quaternionf bbox_quaternion(eigenvectors);
        const Eigen::Vector3f bbox_position = eigenvectors * mean_diag + centroid.head<3>();

#if !DISABLE_DRAWINGS
        auto marker = visualization_msgs::Marker();
        pcl_conversions::fromPCL(cloud->header, marker.header);
        marker.ns = "table_candidates";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::CUBE; // It's called cube, but it can be a rectangular prism
        marker.pose.position.x = bbox_position(0);
        marker.pose.position.y = bbox_position(1);
        marker.pose.position.z = bbox_position(2);
        marker.pose.orientation.x = bbox_quaternion.x();
        marker.pose.orientation.y = bbox_quaternion.y();
        marker.pose.orientation.z = bbox_quaternion.z();
        marker.pose.orientation.w = bbox_quaternion.w();
        marker.scale.x = max_pt.x - min_pt.x;
        marker.scale.y = max_pt.y - min_pt.y;
        marker.scale.z = max_pt.z - min_pt.z;
        decltype(marker.scale.x) max_dimension;
        decltype(marker.scale.x) second_dimension;
        std::tie(max_dimension, second_dimension) = largest_two(marker.scale.x, marker.scale.y, marker.scale.z);
        const auto area = max_dimension * second_dimension;
        ROS_DEBUG_STREAM("Candidate with dimensions " << marker.scale.x << ", " << marker.scale.y << ", " << marker.scale.z << " and area " << area);
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        const auto alpha = 1 - std::abs(1.71 - area) / 3;
        marker.color.a = std::min(1., std::max(0., 0.5 * alpha));

        markers.markers.push_back(marker);

        ROS_DEBUG_STREAM("Added marker with opacity " << alpha << " (rendered " << marker.color.a << ")");
#endif
    }

    marker_pub_.publish(markers);

}

bool table_detector::findTable(geometry_msgs::Point &detected_pt) {
    ROS_INFO_ONCE("findTable called");
    ROS_DEBUG_ONCE("findTable called");
    return false; // TODO
}

table_detector::~table_detector()
{
    pcl_sub_.shutdown();
}
