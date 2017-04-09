#include <iostream>

#include <ros/ros.h>

#include <math.h>

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
#include <pcl/io/pcd_io.h>
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

#include <visualization_msgs/MarkerArray.h>

class sdc{
private:

  ros::Subscriber pcl_sub;

  ros::Publisher pcl_filtered_pub;
  ros::Publisher vis_pub;
  ros::Publisher vis_plane_pub;

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

  void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  void panelRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose);

public:
  // Constructor

  sdc(ros::NodeHandle nh)
  {
    pcl_sub =  nh.subscribe("/assembled_cloud2", 10, &sdc::cloudCB, this);
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_filter/filteredPointCloud", 1);

    vis_pub = nh.advertise<visualization_msgs::Marker>( "/sdc/visualization_marker", 1 );
    vis_plane_pub = nh.advertise<visualization_msgs::Marker>( "/sdc/visualization_plane_vector", 1 );
  }

};
