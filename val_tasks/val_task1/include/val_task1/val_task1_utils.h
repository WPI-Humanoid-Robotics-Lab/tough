#pragma once

#include <ros/ros.h>
#include <srcsim/Satellite.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <val_common/val_common_names.h>

class task1Utils {
private:
    ros::NodeHandle nh_;
    ros::Subscriber satellite_sub_;
    srcsim::Satellite msg_;

    ros::Publisher marker_pub_;
    void satelliteMsgCB (const srcsim::Satellite &msg);

public:
    task1Utils(ros::NodeHandle nh);
    ~task1Utils();

    bool isPitchCorrectNow(void);
    bool isYawCorrectNow(void);
    bool isPitchCompleted(void);
    bool isYawCompleted(void);
    double getPitchDiff (void);
    double getYawDiff (void);
    void getCircle3D (geometry_msgs::Point center, geometry_msgs::Point start,geometry_msgs::Pose pose, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points, float radius, int steps =10);
    void visulatise6DPoints(std::vector<geometry_msgs::Pose> &points);
    void moveToWalkSafePose(void);
};
