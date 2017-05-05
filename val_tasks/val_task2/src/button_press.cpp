#include <val_task2/button_press.h>

button_press::button_press(ros::NodeHandle& nh) : nh_(nh), armTraj_(nh), bd_(nh)
{

}

bool button_press::pressButton(geometry_msgs::Pose& pt)
{
    armSide side = LEFT;
    armTraj_.moveArmInTaskSpace(side, pt, 3.0);
    return true;
}

