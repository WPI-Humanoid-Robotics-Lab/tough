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
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include "val_controllers/robot_state.h"
#include <visualization_msgs/MarkerArray.h>

class SolarPanelDetect
{
private:

  ros::Subscriber pcl_sub;
  ros::Publisher pcl_filtered_pub;
  ros::Publisher vis_pub;


  RobotStateInformer* robot_state_;

  float optimal_dist;
  geometry_msgs::Point button_loc_;
  geometry_msgs::Pose2D rover_loc_;
  bool isroverRight_;
  float rover_theta;
  geometry_msgs::Pose current_pelvis_pose;

  std::vector<geometry_msgs::Pose> detections_;
  int detection_tries_;

  float min_x,max_x, min_y,max_y,min_z,max_z;
  float pitch;


  void cloudCB(const sensor_msgs::PointCloud2::Ptr &input);
  void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void setRoverTheta();
  void visualizept(geometry_msgs::Pose pose);
  void visualizept(float x,float y,float z);
//  float getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients);

  void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float theta, bool isinverse);
  void filter_solar_panel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::Pose &pose);
  void getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::Pose &pose);
  void getOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, geometry_msgs::Pose &pose);
  void boxfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
//  void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients );

public:
  SolarPanelDetect(ros::NodeHandle nh, geometry_msgs::Pose2D rover_loc, bool isroverRight);
  SolarPanelDetect(ros::NodeHandle nh, geometry_msgs::Pose2D rover_loc, bool isroverRight, geometry_msgs::Point button_loc);
  ~SolarPanelDetect();
  bool getDetections(std::vector<geometry_msgs::Pose> &ret_val);
  int getDetectionTries() const;
  void setoffset(float minX=0, float maxX=1.0, float minY=-1.5, float maxY=1.5, float minZ=0.8, float maxZ=1.2, float pitchDeg=0);
  void getoffset(float &minX, float &maxX,float &minY, float &maxY,float &minZ, float &maxZ, float &pitchDeg);
  static void invertYaw(geometry_msgs::Pose &pose);

};
