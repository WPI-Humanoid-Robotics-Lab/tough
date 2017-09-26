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
#include "tough_controller_interface/robot_state.h"
#include <visualization_msgs/MarkerArray.h>
//#include "octomap_server/"

class SolarArrayDetector{
private:

  ros::Subscriber pcl_sub;

  ros::Publisher pcl_filtered_pub;
  ros::Publisher rover_cloud_pub;
  ros::Publisher dbg_points_pub_;


  ros::Publisher vis_pub_array ;

  RobotStateInformer* robot_state_;

  geometry_msgs::Pose2D rover_loc_;
  std::vector<geometry_msgs::Pose> detections_;
  int detection_tries_;
  bool isroverRight_;


  void cloudCB(const sensor_msgs::PointCloud2::Ptr &input);

  void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void planeDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose);

  void roverremove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);


  /*
  void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void removeWalkWay(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool setneg = false);

  pcl::ModelCoefficients::Ptr getCylinderCoefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& lowerBoxCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& upperBoxCloud, geometry_msgs::Pose& pose);
*/
  // void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  // void getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose);

public:
  // Constructor

  SolarArrayDetector(ros::NodeHandle nh, geometry_msgs::Pose2D rover_loc, bool isroverRight);
  ~SolarArrayDetector();
  bool getDetections(std::vector<geometry_msgs::Pose> &ret_val);

  int getDetectionTries() const;


};
