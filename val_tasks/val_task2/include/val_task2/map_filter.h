#ifndef MAP_FILTER_H
#define MAP_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct Point2D{
    int x;
    int y;

};

inline bool operator == (const Point2D& lhs, const Point2D& rhs){
    return (abs(lhs.x - rhs.x) < 5 || abs(lhs.y - rhs.y) < 5);
}

inline bool operator<(const Point2D& lhs, const Point2D& rhs)
{
    return lhs.x < rhs.x || lhs.y < rhs.y;
}

class map_filter
{
public:
    map_filter(ros::NodeHandle& );
    ~map_filter();
    //bool getFinishBoxCenters(std::vector<geometry_msgs::Point> &centers);
    void convertMap(nav_msgs::OccupancyGrid::Ptr msg, uint);

private:
    void mapCB(const nav_msgs::OccupancyGrid::Ptr msg);
    void projectedMapCB(const nav_msgs::OccupancyGrid::Ptr msg);
    void showImage(cv::Mat, std::string caption="MapFiltering");
    ros::NodeHandle nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber projectedMapSub_;
    ros::Publisher  mapPub_;
    nav_msgs::OccupancyGrid occGrid_;
    cv::Mat map_image_;
    cv::Mat projected_map_image_;
    std::set<Point2D> finish_box_centers_;

    float MAP_RESOLUTION;
    float MAP_HEIGHT;
    float MAP_WIDTH;
    float MAP_X_OFFSET;
    float MAP_Y_OFFSET;
};

#endif // MAP_FILTER_H
