#ifndef HANDLE_DETECTOR_H
#define HANDLE_DETECTOR_H

#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisensePointCloud.h>
#include <tough_perception_common/PointCloudHelper.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

class HandleDetector
{
    cv::Mat current_image_, current_image_HSV_, current_disparity_, qMatrix_;
    cv::Mat imRed_, imBlue_, imGray_, imOrange_;
    cv::Mat imRedReduced_, imBlueReduced_;
    cv::Rect roiRed_, roiBlue_, roiOrange_;

    std::vector<cv::Point> rectCenter_;
    std::vector<std::vector<cv::Point> > convexHulls_;
    std::vector< pcl::PointXYZ> buttonCenters_;

    int thresh_ = 100;
    const int hsvGray_[6] = {0, 255, 0, 20, 8, 140}; // lh, hh, ls, hs, lv, hv
    const int hsvOrange_[6] = {11, 25, 165, 255, 140, 255};
    const int hsvRed_[6] = {0, 10, 50, 255, 60, 255};
    const int hsvBlue_[6] = {100, 125, 100, 255, 0, 255};

    int frameID_ = 0;
    std::string side_;

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    tough_perception::MultisenseImage ms_sensor_;
    tough_perception::StereoPointCloudColor::Ptr organizedCloud_;
    visualization_msgs::MarkerArray markers_;
    void visualize_point(geometry_msgs::Point point);
public:

    void showImage(cv::Mat, std::string caption="Handle Detection");
    inline void colorSegment(const cv::Mat &imgHSV, const int[], cv::Mat &outImg);
    void doMorphology(cv::Mat &image);
    void findMaxContour(const cv::Mat, cv::Rect &roi);
    bool findAllContours (const cv::Mat &);
    bool getHandleLocation(std::vector<geometry_msgs::Point> &handleLocs);
    void getReducedImage(cv::Mat &, const cv::Rect &);
    bool findHandles(std::vector<geometry_msgs::Point>&);
    HandleDetector(ros::NodeHandle nh);

};

#endif // HANDLE_DETECTOR_H




