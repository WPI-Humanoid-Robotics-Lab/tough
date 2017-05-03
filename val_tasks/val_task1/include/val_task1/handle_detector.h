#ifndef HANDLE_DETECTOR_H
#define HANDLE_DETECTOR_H

#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_broadcaster.h>//

#include <iostream>
#include <vector>

class handle_detector
{
    cv::Mat current_image_, current_image_HSV_, current_disparity_, qMatrix_;
    cv::Mat imRed_, imBlue_, imGray_, imOrange_;
    cv::Mat imRedReduced_, imBlueReduced_;
    cv::Rect roiRed_, roiBlue_, roiOrange_;

    std::vector<cv::Point> rectCenter;
    std::vector< pcl::PointXYZRGB> buttonCenters_;

    int thresh_ = 100;
    const int hsvGray_[6] = {0, 255, 0, 20, 8, 140}; // lh, hh, ls, hs, lv, hv
    const int hsvOrange_[6] = {11, 25, 165, 255, 140, 255};
    const int hsvRed_[6] = {0, 10, 50, 255, 60, 255};
    const int hsvBlue_[6] = {100, 125, 100, 255, 0, 255};

    int frameID_ = 0;
    std::string side_;

    ros::NodeHandle nh_;
    src_perception::MultisenseImage ms_sensor_;

public:

    void showImage(cv::Mat);
    void colorSegment(const cv::Mat &imgHSV, const int[], cv::Mat &outImg);
    void doMorphology(cv::Mat &image);
    void findMaxContour(const cv::Mat, cv::Rect &roi);
    bool findAllContours (const cv::Mat &);
    bool getHandleLocation(std::vector<geometry_msgs::Point> &handleLocs);
    void getReducedImage(cv::Mat &, const cv::Rect &);
    bool findHandles(std::vector<geometry_msgs::Point>&);
    handle_detector(ros::NodeHandle nh);

};

#endif // HANDLE_DETECTOR_H




