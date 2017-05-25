#ifndef CABLE_DETECTOR_H
#define CABLE_DETECTOR_H

#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>

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

#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include "val_controllers/robot_state.h"

#include <iostream>
#include <vector>
#include <mutex>

class CableDetector
{
    cv::Mat current_image_, current_image_HSV_, current_disparity_, qMatrix_;

    int thresh_ = 100;

                    // lh, uh, ls, us, lv, uv
    // Blue Range    - 114, 125, 100, 255, 0, 255
    // Low Red Range - 0, 5, 178, 255, 51, 149
    // High Red Range - 170, 180, 204, 255, 140, 191
    // mahima range - 23, 39, 172, 255, 104, 205
    int hsv_[6] = {114, 125, 100, 255, 0, 255};

    int frameID_ = 0;
    std::string side_;

    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher pcl_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    ros::Publisher marker_pub_;
    src_perception::MultisenseImage ms_sensor_;
    src_perception::StereoPointCloudColor::Ptr organizedCloud_;
    visualization_msgs::MarkerArray markers_;
    geometry_msgs::Point cableLoc_;
    std::mutex mtx_;

    void visualize_direction(geometry_msgs::Point point1, geometry_msgs::Point point2);
    void visualize_point(geometry_msgs::Point point, double r, double g, double b);

public:
    CableDetector(ros::NodeHandle nh);
    ~CableDetector();

    void setTrackbar();
    void showImage(cv::Mat, std::string caption="Cable Detection");
    void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2);
    void colorSegment(cv::Mat &imgHSV, cv::Mat &outImg);

    size_t findMaxContour(const std::vector<std::vector<cv::Point> >& contours);
    bool getCableLocation(geometry_msgs::Point &);
    std::vector<cv::Point> getOrientation(const std::vector<cv::Point> &, cv::Mat &);
    bool findCable(geometry_msgs::Point &);

    void cloudCB(const sensor_msgs::PointCloud2::Ptr& );
    bool planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

};

#endif // CABLE_DETECTOR_H
