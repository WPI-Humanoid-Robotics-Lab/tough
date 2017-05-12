#ifndef PANEL_DETECTION_H
#define PANEL_DETECTION_H

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
#include "val_common/val_common_names.h"

struct PanelSettings{

    std::string settingName;
    float x_max_limit;
    float x_min_limit;

    float y_max_limit;
    float y_min_limit;

    float z_max_limit;
    float z_min_limit;

    float OFFSET;
};

enum class DETECTOR_TYPE{
    HANDLE_PANEL_COARSE = 0,
    HANDLE_PANEL_FINE
};

class PanelDetector{
private:

    ros::Subscriber pcl_sub_;

    ros::Publisher pcl_filtered_pub_;
    ros::Publisher vis_pub_;
    ros::Publisher vis_plane_pub_;

    std::vector<geometry_msgs::Pose> detections_;

    int detection_tries_;

    void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

    void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void panelSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose);

    std::vector<float> panel_plane_model_;
    std::map<DETECTOR_TYPE, PanelSettings> preset_configs_;
    void setPresetConfigs();
//    PanelSettings* currentSettings_;
    DETECTOR_TYPE currentDetector;

public:
    // Constructor

    PanelDetector(ros::NodeHandle &nh, DETECTOR_TYPE detector_type);
    ~PanelDetector();

    void getDetections(std::vector<geometry_msgs::Pose> &ret_val);

    int getDetectionTries() const;
    void setDetectionTries(int getDetectionTries);
    bool getPanelPlaneModel(std::vector<float> &panelPlaneModel) const;
};

//bool poseComparator (geometry_msgs::Pose const& lhs, geometry_msgs::Pose const& rhs)
//{
//    tf::Pose tf_lhs, tf_rhs;
//    tf::poseMsgToTF(lhs, tf_lhs);
//    tf::poseMsgToTF(rhs, tf_rhs);
//    return tf_lhs.getOrigin() < tf_rhs.getOrigin();

//}

#endif //PANEL_DETECTION_H
