#pragma once

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CartesianPlanner {
private:
    std::string group_name_;
    std::string reference_frame_;

    // planning group
    moveit::planning_interface::MoveGroupInterfacePtr group_;

public:
    CartesianPlanner(std::string group_name, std::string reference_frame="/world");
    ~CartesianPlanner();
    double getTrajFromCartPoints(std::vector<geometry_msgs::Pose>& points, moveit_msgs::RobotTrajectory& trajectory, bool avoid_collisions=true, float goal_tolerance=0.1f);

};
