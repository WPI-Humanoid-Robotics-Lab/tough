#ifndef STEPS_DETECTOR_H
#define STEPS_DETECTOR_H

#include <iostream>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include "val_controllers/robot_state.h"
#include <visualization_msgs/MarkerArray.h>
#include "val_task3/stair_detector.h"

class steps_detector
{
    ros::Subscriber pcl_sub_;
    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    stair_detector sd_;

public:
    steps_detector(ros::NodeHandle& );
    ~steps_detector();
    void stepsCB(const sensor_msgs::PointCloud2::Ptr& );
    void planeSegmentation(const std::vector<double>& , const geometry_msgs::Point & );
    struct less_than_key
    {
        inline bool operator ()(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
        {
        return ((std::pow(p1.x, 2) + std::pow(p1.y, 2) + std::pow(p1.z, 2)) < (std::pow(p2.x, 2) + std::pow(p2.y, 2) + std::pow(p2.z, 2)));
        }
    };
};

#endif // STEPS_DETECTOR_H
