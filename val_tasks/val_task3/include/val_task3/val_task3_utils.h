#ifndef VAL_TASK3_UTILS_H
#define VAL_TASK3_UTILS_H

#include <ros/ros.h>
#include <srcsim/Satellite.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <val_common/val_common_names.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>
#include <val_controllers/val_gripper_control.h>
#include <val_controllers/val_head_navigation.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/robot_state.h>
#include "navigation_common/map_generator.h"
#include <srcsim/Task.h>
#include <mutex>

class task3Utils{
private:
    ros::NodeHandle nh_;
    armTrajectory arm_controller_;
    srcsim::Task task_msg_;

    bool is_climbstairs_finished_;
    std::mutex climstairs_flag_mtx_;

    const std::vector<float> RIGHT_ARM_SEED_TABLE_MANIP = {-0.23, 0, 0.70, 1.51, -0.05, 0, 0};
    const std::vector<float> LEFT_ARM_SEED_TABLE_MANIP  = {-0.23, 0, 0.70, -1.51, 0.05, 0, 0};
    const std::vector<float> RIGHT_ARM_DOOR_OPEN = {-1.35, 1.20, 0.75, 0.50, 1.28, 0.0, 0.0};
    const std::vector<float> LEFT_ARM_DOOR_OPEN  = {-0.2f, -1.2f, 0.7222f, -1.5101f, 0.0f, 0.0f, 0.0f};

    // Visited map
    ros::Subscriber visited_map_sub_, task_status_sub_;
    void visited_map_cb(const nav_msgs::OccupancyGrid::Ptr msg);
    void taskStatusCB(const srcsim::Task &msg);
    nav_msgs::OccupancyGrid visited_map_;

public:
    task3Utils(ros::NodeHandle nh);
    ~task3Utils();
    void beforePanelManipPose();
    void beforDoorOpenPose();
    void blindNavigation(geometry_msgs::Pose2D &goal);

    bool isClimbstairsFinished() const;
    void resetClimbstairsFlag(void);
};






















#endif // VAL_TASK3_UTILS_H
