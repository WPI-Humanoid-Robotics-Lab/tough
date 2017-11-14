#ifndef BUTTON_DETECTOR_H
#define BUTTON_DETECTOR_H

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
#include <tf/transform_listener.h>
#include <tough_common/val_common_names.h>

#include <iostream>
#include <vector>

//red button only
class ButtonDetector
{
    cv::Mat current_image_, current_image_HSV_, current_disparity_, qMatrix_;
    cv::Mat imLowRed_, imHighRed_;

    int thresh_ = 100;
    const int hsvLowRed_[6] = {0, 5, 178, 255, 51, 149}; // lh, uh, ls, us, lv, uv
    const int hsvHighRed_[6] = {170, 180, 204, 255, 140, 191};

    std::vector<cv::Point> convexHulls_;

    int frameID_ = 0;

    ros::NodeHandle nh_;
    tough_perception::MultisenseImage* ms_sensor_;

    ros::Publisher marker_pub_;
    visualization_msgs::MarkerArray markers_;
    void visualize_point(const geometry_msgs::Point &point);
public:

    void showImage(cv::Mat, std::string caption="Button Detection");
    inline void colorSegment(const cv::Mat &imgHSV, const int[], const int[], cv::Mat &outImg);
    size_t findMaxContour(const std::vector<std::vector<cv::Point> >&);
    bool getButtonLocation(geometry_msgs::Point &buttonLoc);
    bool findButtons(geometry_msgs::Point&);
    ButtonDetector(ros::NodeHandle nh, tough_perception::MultisenseImage* ms_sensor);
    ~ButtonDetector();

};

#endif // BUTTON_DETECTOR_H





