# pragma once

#include <ros/ros.h>
#include <val_footstep/RobotWalker.h>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/robot_state.h>

#define FIRSTSTEP_OFFSET    0.3
#define STEP_HEIGHT         0.3    // 0.2031 actual height
#define STEP_DEPTH          0.25   // 0.2389 actual depth

#define DEFAULT_SWINGHEIGHT 0.18

#define APPROACH 1

class climbStairs {
private:
    ros::NodeHandle nh_;
    RobotWalker *walker_;
    chestTrajectory *chest_;
    pelvisTrajectory *pelvis_;
    armTrajectory *arm_;
    RobotStateInformer *current_state_;

    int approach_ ;
    void approach_1 (void);
    void approach_2 (void);

public:
    climbStairs(ros::NodeHandle nh);
    ~climbStairs();
    void climb_stairs (void);
};


