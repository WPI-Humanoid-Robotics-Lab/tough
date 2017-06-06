#ifndef STAIR_DETECTOR_2_H
#define STAIR_DETECTOR_2_H

#include <geometry_msgs/Point.h>

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

class stair_detector_2
{
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher points_pub_;
    ros::Publisher blacklist_pub_;
    ros::Subscriber pcl_sub_;

    tf::TransformListener tf_listener_;

    src_perception::MultisensePointCloud point_cloud_listener_;

    std::vector<std::vector<geometry_msgs::Pose>> detections_;

public:

    typedef pcl::PointXYZ                   Point;
    typedef pcl::PointCloud<Point>          PointCloud;
    typedef pcl::Normal                     Normal;
    typedef pcl::PointCloud<Normal>         NormalCloud;
    typedef pcl::PointXYZL                  LabeledPoint;
    typedef pcl::PointCloud<LabeledPoint>   LabeledCloud;

    stair_detector_2(ros::NodeHandle nh);

    void cloudCB(const stair_detector_2::PointCloud::ConstPtr &cloud);
    std::vector<std::vector<geometry_msgs::Pose>> getDetections() const;

    ~stair_detector_2();

private:
    PointCloud::Ptr prefilterCloud(const PointCloud::ConstPtr &cloud_raw) const;
    NormalCloud::Ptr estimateNormals(const PointCloud::ConstPtr &filtered_cloud) const;
    bool estimateStairs(const PointCloud::ConstPtr &filtered_cloud,
                            const NormalCloud::ConstPtr &filtered_normals);

    size_t estimateStairPose(const PointCloud::ConstPtr &filtered_cloud,
                             const std::vector<pcl::PointIndices> &step_clusters,
                             const Eigen::Vector3f &stairs_dir, Eigen::Affine3f &stairs_pose) const;

    std::vector<pcl::PointIndices> findStairClusters(const Eigen::Vector3f &avg_normal,
                                                     const PointCloud::ConstPtr &filtered_cloud) const;

    Eigen::Vector3f modelToVector(const pcl::ModelCoefficients &model) const;

};
#endif // STAIR_DETECTOR_2_H
