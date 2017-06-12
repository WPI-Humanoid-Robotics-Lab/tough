# pragma once

#include <ros/ros.h>
#include <val_footstep/ValkyrieWalker.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>
#include <val_controllers/val_arm_navigation.h>

#define FIRSTSTEP_OFFSET    0.3
#define STEP_HEIGHT         0.3    // 0.2031 actual height
#define STEP_DEPTH          0.25   // 0.2389 actual depth

#define DEFAULT_SWINGHEIGHT 0.18


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


