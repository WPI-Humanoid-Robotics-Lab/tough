#include <val_task2/button_press.h>

button_press::button_press(ros::NodeHandle& nh) : nh_(nh), armTraj_(nh), bd_(nh)
{
    bd_.findButtons(point_);
}

bool button_press::pressButton(geometry_msgs::Point& pt)
{
    armSide side = RIGHT;
    geometry_msgs::Pose point;
    ROS_INFO_STREAM(pt.x<<"\t"<<pt.y<<"\t"<<pt.z<<std::endl);
    point.position.x = pt.x;
    point.position.y = pt.y;
    point.position.z = pt.z + 0.4;
    point.orientation.x = 0;
    point.orientation.y = 0;
    point.orientation.z = 0;
    point.orientation.w = 1;
    ROS_INFO_STREAM(point.position.x<<"\t"<<point.position.y<<"\t"<<point.position.z<<std::endl);
    armTraj_.moveArmInTaskSpace(side,point,3.0);
    ros::Duration(3).sleep();
    geometry_msgs::Pose point1;
    point1.position.x = pt.x;
    point1.position.y = pt.y;
    point1.position.z = pt.z;
    point1.orientation.x = 0;
    point1.orientation.y = 0;
    point1.orientation.z = 0;
    point1.orientation.w = 1;
    armTraj_.moveArmInTaskSpace(side,point1,3.0);
    ros::Duration(3).sleep();
    armTraj_.moveArmInTaskSpace(side, point, 3.0);
    ros::Duration(3).sleep();
    armTraj_.moveToDefaultPose(side);
    ros::Duration(3).sleep();
    ROS_INFO_STREAM("done" << std::endl);
    return true;
}

bool button_press::getPressButton()
{
    pressButton(point_);
    return true;
}


