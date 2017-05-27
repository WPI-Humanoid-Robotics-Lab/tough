#include <iostream>
#include <math.h>
#include <memory>

#include <ros/ros.h>

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>

#include "val_controllers/robot_state.h"
#include <visualization_msgs/MarkerArray.h>

class DoorValvedetector{
private:

  ros::Subscriber pcl_sub_;

  ros::Publisher pcl_filtered_pub_;

  ros::Publisher vis_pub_;
  RobotStateInformer* robot_state_;

  float min_x,max_x,min_y,max_y,min_z,max_z;





  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
  void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void setOffset(float minX = 0.2, float maxX = 2.0,float minY = -0.7 , float maxY = 0.7 ,float minZ= 0.5, float maxZ=1.5);
  void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool isinverse);

public:

  // Constructor

  DoorValvedetector(ros::NodeHandle nh);
  ~DoorValvedetector();

};
