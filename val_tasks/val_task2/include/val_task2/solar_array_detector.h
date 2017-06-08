#ifndef SOLAR_ARRAY_DETECTOR_H
#define SOLAR_ARRAY_DETECTOR_H

//Standard Library
#include <iostream>
#include <math.h>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//Rviz
#include <visualization_msgs/MarkerArray.h>

//PCL
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
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>

#include "val_controllers/robot_state.h"


#define ARRAY_OFFSET 0.96    //Offset to get the correct stand pose near array

class CoarseArrayDetector{
private:

  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher rover_cloud_pub_;

  int frameID_ = 0;
  visualization_msgs::MarkerArray markers_;

  RobotStateInformer* robot_state_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  geometry_msgs::Pose2D rover_pose_;
  geometry_msgs::Pose pose_;

  std::mutex mtx_;

  void cloudCB(const sensor_msgs::PointCloud2::Ptr input);

  void visualizePose(const geometry_msgs::Pose& pose);

  void roverRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  void normalSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  void findArrayCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  bool planeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, pcl::ModelCoefficients::Ptr coefficients);
  void standPose(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::ModelCoefficients::Ptr coefficients);

public:
  // Constructor

  CoarseArrayDetector(ros::NodeHandle& nh);
  ~CoarseArrayDetector();

  bool getArrayPosition(const geometry_msgs::Pose2D &rover_pose, geometry_msgs::Pose &array_pose);
};

#endif // SOLAR_ARRAY_DETECTOR_H
