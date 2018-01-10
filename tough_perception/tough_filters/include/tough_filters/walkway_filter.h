#ifndef WALKWAY_FILTER_H
#define WALKWAY_FILTER_H

#define LOWER_THRESHOLD -0.00f
#define UPPER_THRESHOLD  0.07f
#define GROUND_THRESHOLD (UPPER_THRESHOLD - LOWER_THRESHOLD)
#define FOOT_GROUND_THRESHOLD 0.05

#include <ros/ros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "tough_common/robot_description.h"

class WalkwayFilter{
public:
    WalkwayFilter(ros::NodeHandle &n);
    ~WalkwayFilter();
    void generateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr msg);
    void subtractPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices::Ptr outliers);

private:
    ros::Publisher mapPub_;
    ros::Publisher pointcloudPub_;
    ros::Subscriber pointcloudSub_;
    ros::NodeHandle nh_;
    tf::TransformListener       tf_listener_;
    RobotDescription* rd_;
    double getCurrentFootHeight(void);
};

#endif // WALKWAY_FILTER_H

