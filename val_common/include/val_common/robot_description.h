#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <string>
#include <ros/ros.h>
#include <urdf/model.h>

class RobotDescription
{
public:
    RobotDescription(ros::NodeHandle nh, std::string urdf_param="/robot_description");
    std::string PELVIS_TF;

private:
    urdf::Model model_;
    std::vector<urdf::JointSharedPtr> joints_;
    std::vector<urdf::LinkSharedPtr> links_;
    std::string robot_name_;
};

#endif // ROBOT_DESCRIPTION_H
