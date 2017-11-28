#pragma once

#include <ros/ros.h>
#include <srcsim/Satellite.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tough_common/val_common_names.h>
#include <chrono>
#include <ctime>
#include <std_msgs/Empty.h>
#include <srcsim/Task.h>
#include "ros/package.h"
#include <fstream>
#include <cstdlib>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "navigation_common/map_generator.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <tough_footstep/RobotWalker.h>
#include <srcsim/Harness.h>

// minimum moment to determine the rotation direction
#define HANDLE_MINIMUM_MOVMENT_IN_RAD      0.0698132 // 4 deg
#define HANDLE_CONSTANT_THRESHOLD_IN_RAD   0.005     // ~0.3 deg  (// tricky to choose the value, it depends on body acceleration)
#define HANDLE_CONSTANT_DEBOUNCE_TIME_SEC  5

// control time ouit (real time)
#define HANDLE_CONTROL_TIMEOUT_SEC 60 //sec

// handle positions
//!!! enum is not used as this will be used for vector acess
#define PITCH_KNOB_CENTER    0
#define PITCH_KNOB_HANDLE    1
#define YAW_KNOB_CENTER      2
#define YAW_KNOB_HANDLE      3

// previous grasp state
enum class prevGraspState {
    NOT_INITIALISED = 0,
    GRASP_PITCH_HANDLE,
    GRASP_YAW_HANDLE
};

// pitch yaw selection
enum class controlSelection {
    CONTROL_NOT_INITIALISED = 0,
    CONTROL_PITCH ,
    CONTROL_YAW
};

// Pitch/Yaw direction
enum class valueDirection {
    VALUE_NOT_INITIALISED = 0,
    VALUE_CONSTANT,
    VALUE_TOWARDS_TO_GOAL,
    VALUE_AWAY_TO_GOAL,
    VALUE_TOGGLING,
    VALUE_INCRSING,
    VALUE_DECRASING
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
    ros::Subscriber task_status_sub_;
    RobotStateInformer* current_state_;
    RobotDescription *rd_;

    ros::Publisher marker_pub_, clearbox_pointcloud_pub_, reset_pointcloud_pub_, task1_log_pub_, reset_map_pub_;
    void satelliteMsgCB (const srcsim::Satellite &msg);
    int current_checkpoint_;
    srcsim::Task taskMsg;

    std::mutex mtx, harness_mtx_;
    nav_msgs::OccupancyGrid visited_map_;
    void visitedMapUpdateCB(const nav_msgs::OccupancyGrid &msg);
    ros::Subscriber visitedMapUpdaterSub_, harness_sub_;
    RobotWalker* walk_;
    void harnessCB(const srcsim::Harness &harnessMsg);
    bool is_harness_detached_;



public:
    const std::string TEXT_RED="\033[0;31m";
    const std::string TEXT_GREEN = "\033[0;32m";
    const std::string TEXT_NC=  "\033[0m";

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
    valueDirection getValueStatus(double current_value, controlSelection control);
    void getCircle3D (geometry_msgs::Point center, geometry_msgs::Point start,geometry_msgs::Quaternion orientation, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points, handleDirection direction, float radius, int steps =10);
    void visulatise6DPoints(std::vector<geometry_msgs::Pose> &points);
    void clearBoxPointCloud();
    void resetPointCloud();
    void clearMap();
    void taskStatusSubCB(const srcsim::Task &msg);
    void terminator(const ros::TimerEvent& t);
    void fixHandleArray(std::vector<geometry_msgs::Point> &handle_loc, std::vector<geometry_msgs::Point> &pclHandlePoses);
    int getCurrentCheckpoint() const;
    void reOrientTowardsGoal(geometry_msgs::Point goal_point, float offset=0.0f);
    bool isPointVisited(float x, float y);
    bool getNextPoseToWalk(geometry_msgs::Pose2D &pose2D, bool allowVisitied = false);

    boost::posix_time::ptime timeNow;
    std::string logFile;
    ros::Timer timer_;

    valueDirection getGoalDirection(double current_value, controlSelection control);
    void taskLogPub(std::string data);

    bool isHarnessDetached() const;
    void setHarnessDetached(bool isHarnessDetached);
};
