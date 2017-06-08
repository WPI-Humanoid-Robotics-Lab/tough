#ifndef SOLAR_PANEL_DETECTOR_2_H
#define SOLAR_PANEL_DETECTOR_2_H

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

class solar_panel_detector_2
{
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher points_pub_;
    ros::Publisher blacklist_pub_;
    ros::Subscriber pcl_sub_;

    tf::TransformListener tf_listener_;

    src_perception::MultisensePointCloud point_cloud_listener_;

    geometry_msgs::PoseStamped rover_pose_;

    bool detection_success_;
    geometry_msgs::PoseStamped latest_detection_;

public:

    typedef pcl::PointXYZ                   Point;
    typedef pcl::PointCloud<Point>          PointCloud;
    typedef pcl::Normal                     Normal;
    typedef pcl::PointCloud<Normal>         NormalCloud;
    typedef pcl::PointXYZL                  LabeledPoint;
    typedef pcl::PointCloud<LabeledPoint>   LabeledCloud;

    solar_panel_detector_2(ros::NodeHandle nh, const geometry_msgs::PoseStamped &rover_pose);

    void setRoverPose(const geometry_msgs::PoseStamped &pos);
    geometry_msgs::PoseStamped getRoverPose() const;

    void cloudCB(const solar_panel_detector_2::PointCloud::ConstPtr &cloud);

    bool getPanelPose(geometry_msgs::PoseStamped &pose) const;

    ~solar_panel_detector_2();

private:
    PointCloud::Ptr prefilterCloud(const PointCloud::ConstPtr &cloud_raw) const;
    NormalCloud::Ptr estimateNormals(const PointCloud::ConstPtr &filtered_cloud) const;

    bool findPointsInsideTrailer(const PointCloud::ConstPtr &points,
                                 const NormalCloud::Ptr &normals, // not ConstPtr because PCL demands a Ptr
                                 pcl::PointIndices &pts_inside_trailer) const;

    bool findSolarPanelPose(const PointCloud::ConstPtr &points,
                            const NormalCloud::Ptr &normals, // not ConstPtr because of PCL
                            const pcl::PointIndices::ConstPtr &pts_inside_trailer,
                            geometry_msgs::PoseStamped &solar_panel_pose) const;

    float findGapAlongAxis(const PointCloud::ConstPtr &points, const int axis) const;

    void publishClusters(const PointCloud::ConstPtr &cloud, const std::vector<pcl::PointIndices> &clusters) const;
    void publishNormals(const PointCloud &points, const NormalCloud &normals) const;
    Eigen::Vector3f modelToVector(const pcl::ModelCoefficients &model) const;

};
#endif // SOLAR_PANEL_DETECTOR_2_H
