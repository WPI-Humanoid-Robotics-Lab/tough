# pragma once

#include <ros/ros.h>
#include <tough_footstep/RobotWalker.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/robot_state.h>

#define FIRSTSTEP_OFFSET    0.3
#define STEP_HEIGHT         0.3    // 0.2031 actual height
#define STEP_DEPTH          0.25   // 0.2389 actual depth

#define DEFAULT_SWINGHEIGHT 0.18

#define APPROACH 1

class climbStairs {
private:
    ros::NodeHandle nh_;
    RobotWalker *walker_;
    ChestControlInterface *chest_;
    PelvisControlInterface *pelvis_;
    ArmControlInterface *arm_;
    RobotStateInformer *current_state_;
    RobotDescription *rd_;

    int approach_ ;
    void approach_1 (void);
    void approach_2 (void);

public:
    climbStairs(ros::NodeHandle nh);
    ~climbStairs();
    void climb_stairs (void);
};


