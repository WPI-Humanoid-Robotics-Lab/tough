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

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include "tough_controller_interface/robot_state.h"
#include <visualization_msgs/MarkerArray.h>
enum class ROVER_SIDE{
    LEFT = 0,
    RIGHT,
    UNKOWN
};

class RoverDetector{
private:

  ros::Subscriber pcl_sub_;

  ros::Publisher pcl_filtered_pub_;

  ros::Publisher vis_pub_;

  ros::Publisher vis_plane_pub_;

  std::vector<std::vector<geometry_msgs::Pose>> detections_;

  int detection_tries_;

  RobotStateInformer* current_state_;

  ROVER_SIDE roverSide_;

  bool finePose_;

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

  void lowerBoxPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void upperBoxPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void planeDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void getPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lowerBoxCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& upperBoxCloud, std::vector<geometry_msgs::Pose>& pose);

public:

  // Constructor

  RoverDetector(ros::NodeHandle nh, bool getFine=false);
  ~RoverDetector();
  bool getDetections(std::vector<std::vector<geometry_msgs::Pose> > &ret_val);

  int getDetectionTries() const;
  bool getRoverSide(ROVER_SIDE &side) const;

};
