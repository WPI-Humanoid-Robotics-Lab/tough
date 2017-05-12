#ifndef PCL_HANDLE_DETECTOR_H
#define PCL_HANDLE_DETECTOR_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

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
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/crop_box.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include "val_common/val_common_names.h"
#include "val_control/robot_state.h"


class pcl_handle_detector
{
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_filtered_pub_;
  ros::Publisher vis_pub_;

  float offset;
  geometry_msgs::Pose panel_coarse_loc_;
  std::vector<geometry_msgs::Pose> handle_loc_;
  RobotStateInformer* robot_state_;

  void cloudCB(const sensor_msgs::PointCloud2::Ptr &input);
  bool extractHandle(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

public:
  pcl_handle_detector(ros::NodeHandle &nh, float panel_offset,geometry_msgs::Pose panel_loc_);
  ~pcl_handle_detector();


};

#endif // HANDLE_DETECTOR_H
