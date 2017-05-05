#ifndef BUTTON_PRESS_H
#define BUTTON_PRESS_H
#include <val_task2/button_detector.h>
#include <val_control/val_arm_navigation.h>


class button_press
{
    armTrajectory armTraj_;
    ros::NodeHandle nh_;
    button_detector bd_;
    geometry_msgs::Pose pt;

public:

    button_press(ros::NodeHandle&);
    bool pressButton(geometry_msgs::Pose&);
};

#endif // BUTTON_PRESS_H
