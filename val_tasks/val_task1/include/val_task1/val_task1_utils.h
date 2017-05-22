#pragma once

#include <ros/ros.h>
#include <srcsim/Satellite.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <val_common/val_common_names.h>


// minimum moment to determine the rotation direction
#define  MINIMUM_MOVMENT_IN_RAD  0.0872665

// previous grasp state
enum class prevGraspState {
    NOT_INITIALISED = 0,
    GRASP_PITCH_HANDLE,
    GRASP_YAW_HANDLE
};

// handle positions
//!!! enum is not used as this will be used for vector acess
#define PITCH_KNOB_CENTER    0
#define PITCH_KNOB_HANDLE    1
#define YAW_KNOB_CENTER      2
#define YAW_KNOB_HANDLE      3

// pitch yaw selection
enum class controlSelection {
    PITCH = 0,
    YAW
};

// Pitch/Yaw direction
enum class valueDirection {
    VALUE_CONSTANT = 0,
    VALUE_INCREASING,
    VALUE_DECREASING
};

enum class valueConstant {
    NOT_INITIALISED = 0,
    VALUE_NOT_CHANGING,
    VALUE_CHANGING
};

enum class handleDirection {
    ANTICLOCK_WISE = 0,
    CLOCK_WISE
};

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
    double getPitch (void);
    double getYaw (void);
    valueDirection getValueDirection(double current_value, controlSelection control);
    void getCircle3D (geometry_msgs::Point center, geometry_msgs::Point start,geometry_msgs::Quaternion orientation, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points, handleDirection direction, float radius, int steps =10);
    void visulatise6DPoints(std::vector<geometry_msgs::Pose> &points);
    valueConstant isValueConstant(double current_value, controlSelection control);
};
