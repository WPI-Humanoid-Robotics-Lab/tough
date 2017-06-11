# pragma once

#include <ros/ros.h>
#include <val_footstep/ValkyrieWalker.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>
#include <val_controllers/val_arm_navigation.h>


class climbStairs {
private:
    ros::NodeHandle nh_;
    ValkyrieWalker *walker_;
    chestTrajectory *chest_;
    pelvisTrajectory *pelvis_;
    armTrajectory *arm_;

public:
    climbStairs(ros::NodeHandle nh);
    ~climbStairs();
    void climb_stairs (void);
};


