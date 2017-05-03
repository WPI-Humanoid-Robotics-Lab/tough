#ifndef FINISH_BOX_DETECTOR_H
#define FINISH_BOX_DETECTOR_H


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv/cv.h>

class FinishBoxDetector{

public:
    FinishBoxDetector(ros::NodeHandle &n);
    ~FinishBoxDetector();

private:

    void detectFinishBox(const nav_msgs::OccupancyGrid::Ptr msg);
    void trimTo2DecimalPlaces(float &x, float &y);
    size_t getIndex(float x, float y);
    ros::NodeHandle nh_;
    ros::Subscriber pointcloudSub_;
    ros::Publisher  mapPub_;
    nav_msgs::OccupancyGrid occGrid_;
    cv::Mat map_image_;

    float MAP_RESOLUTION;
    float MAP_HEIGHT;
    float MAP_WIDTH;
    float MAP_X_OFFSET;
    float MAP_Y_OFFSET;

};

#endif // FINISH_BOX_DETECTOR_H


