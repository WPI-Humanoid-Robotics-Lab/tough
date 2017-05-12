#ifndef STAIR_DETECTOR_H
#define STAIR_DETECTOR_H

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

#include <iostream>
#include <vector>

class stair_detector
{
    cv::Mat current_image_, current_image_HSV_, current_disparity_, qMatrix_;

    int thresh_ = 100;

                    // lh, uh, ls, us, lv, uv
    // Blue Range    - 114, 125, 100, 255, 0, 255
    // Low Red Range - 0, 5, 178, 255, 51, 149
    // High Red Range - 170, 180, 204, 255, 140, 191
    // Golden Yellow Range - 23, 93, 76, 255, 22, 255
    int hsv_[6] = {23, 93, 76, 255, 22, 255};

    std::vector<cv::Point> convexHulls_;


    int frameID_ = 0;
    std::string side_;

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    src_perception::MultisenseImage ms_sensor_;
    src_perception::StereoPointCloudColor::Ptr organizedCloud_;
    visualization_msgs::MarkerArray markers_;
    void visualize_point(geometry_msgs::Point point);

public:
    stair_detector(ros::NodeHandle nh);
    void setTrackbar();
    void showImage(cv::Mat, std::string caption="stair Detection");
    void colorSegment(cv::Mat &imgHSV, cv::Mat &outImg);
    size_t findMaxContour(const std::vector<std::vector<cv::Point> >& contours);
    bool getStairLocation(geometry_msgs::Point &);
    //cv::Point getOrientation(const std::vector<cv::Point> &, cv::Mat &);
    //void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2);
    bool findStair(geometry_msgs::Point &);
    ~stair_detector();

};
#endif // STAIR_DETECTOR_H
