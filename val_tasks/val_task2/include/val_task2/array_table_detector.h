#ifndef ARRAY_TABLE_DETECTOR_H
#define ARRAY_TABLE_DETECTOR_H

#include <geometry_msgs/Point.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/pca.h>

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include "val_controllers/robot_state.h"

#include <iostream>
#include <vector>
#include <mutex>

#define TABLE_OFFSET -0.4

class ArrayTableDetector{
public :
    ArrayTableDetector(ros::NodeHandle nh, geometry_msgs::Point cable_location);
    ~ArrayTableDetector();
    bool planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input);
    void extractCloudOfInterest(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ> &output);
    void getLargestCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void visualizePose(const geometry_msgs::Pose &pose, double r=0, double g=1, double b=1);
    bool getDetections(std::vector<geometry_msgs::Pose> &output);
    void clearDetections();
private:
    void cloudCB(const sensor_msgs::PointCloud2::Ptr &input );
    std::vector<geometry_msgs::Pose> detections_;

    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher pcl_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    ros::Publisher marker_pub_;
    geometry_msgs::Point cable_loc_;
    std::mutex mtx_;
    RobotStateInformer* robot_state_;
};

#endif // ARRAY_TABLE_DETECTOR_H
