#ifndef BUTTON_PRESS_H
#define BUTTON_PRESS_H
#include <val_task2/button_detector.h>
#include <val_control/val_arm_navigation.h>


class button_press
{
    geometry_msgs::Point point_;
    armTrajectory armTraj_;
    ros::NodeHandle nh_;
    button_detector bd_;

public:

    button_press(ros::NodeHandle&);
    bool pressButton(geometry_msgs::Point&);
    bool getPressButton();
};

#endif // BUTTON_PRESS_H
