#ifndef SOLAR_ARRAY_DETECTOR_H
#define SOLAR_ARRAY_DETECTOR_H

#include <iostream>
#include <math.h>

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>

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
#include <pcl/filters/crop_box.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include "val_controllers/robot_state.h"
#include <visualization_msgs/MarkerArray.h>

class ArrayDetector{
private:

  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  ros::Publisher marker_pub_;

  int frameID_ = 0;
  visualization_msgs::MarkerArray markers_;

  RobotStateInformer* robot_state_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  geometry_msgs::Pose2D rover_pose_;
  std::mutex mtx_;

  void cloudCB(const sensor_msgs::PointCloud2::Ptr input);


public:
  // Constructor

  ArrayDetector(ros::NodeHandle& nh);
  ~ArrayDetector();
  void normalSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);

  void roverRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  bool getArrayPosition(const geometry_msgs::Pose2D &rover_pose);
  void boxFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  void findLargestCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  void planeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  void visualizePose(const geometry_msgs::Pose& pose);
};

#endif // SOLAR_ARRAY_DETECTOR_H
