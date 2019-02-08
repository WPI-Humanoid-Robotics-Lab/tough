#ifndef WALKWAY_GENERATOR_H
#define WALKWAY_GENERATOR_H

#include <ros/ros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "tough_common/robot_description.h"

#define SURFACE_NORMAL_THRESHOLD 0.94  // 1.0 for normals perpendicular to z-axis

class WalkwayGenerator
{
public:
  WalkwayGenerator(ros::NodeHandle& n);
  ~WalkwayGenerator();
  void generateWalkwayPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
  ros::Publisher pointcloudPub_;
  ros::Subscriber pointcloudSub_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  RobotDescription* rd_;

  double getCurrentFootHeight(void);
  void zAxisLimitFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud);
  bool getLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  const float CLUSTER_TOLERANCE = 0.058f;
};

#endif  // WALKWAY_GENERATOR_H
