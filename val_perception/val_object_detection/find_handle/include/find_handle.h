#ifndef FIND_HANDLE_H
#define FIND_HANDLE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <perception_common/MultisenseImage.h>
#include <iostream>
#include <vector>

class handle_detector
{
    cv::Mat mi_, imgHSV_;
    cv::Mat imRed_, imBlue_, imGray_;
    cv::Mat imRedReduced_, imBlueReduced_;
    cv::Rect roiRed_, roiBlue_;

    std::vector<cv::Point> rectCenter;

    int thresh_ = 100;
    const int hsvGray_[6] = {0, 255, 0, 20, 0, 140}; // lh, hh, ls, hs, lv, hv
    const int hsvOrange_[6] = {11, 25, 165, 255, 140, 255};
    const int hsvRed_[6] = {0, 10, 50, 255, 60, 255};
    const int hsvBlue_[6] = {100, 125, 100, 255, 0, 255};

public:

    handle_detector(ros::NodeHandle&);
    void showImage(cv::Mat);
    cv::Mat colorSegment(cv::Mat, const int[]);
    cv::Mat doMorphology (cv::Mat);
    cv::Rect findMaxContour (cv::Mat);
    bool findAllContours (cv::Mat);
    void getHandleLocation();
    cv::Mat getReducedImage (cv::Mat, cv::Rect);
    void findHandles(void);

};

#endif // FIND_HANDLE_H




