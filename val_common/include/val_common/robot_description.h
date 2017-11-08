#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <urdf/model.h>

class RobotDescription
{
public:
    static RobotDescription* getRobotDescription(ros::NodeHandle nh, std::string urdf_param="/robot_description");

    std::string getPelvisFrame() const;

    std::string getTorsoFrame() const;

    std::string getLeftFootFrameName() const;

    std::string getRightFootFrameName() const;

    std::string getLeftPalmFrame() const;

    std::string getRightPalmFrame() const;

    void getLeftArmJointNames(std::vector<std::string> &left_arm_joint_names) const;

    void getRightArmJointNames(std::vector<std::string> &right_arm_joint_names) const;

    void getLeftArmFrameNames(std::vector<std::string> &left_arm_frame_names) const;

    void getRightArmFrameNames(std::vector<std::string>  &right_arm_frame_names) const;

    void getLeftArmJointLimits(std::vector<std::pair<float, float> > &left_arm_joint_limits) const;

    void getRightArmJointLimits(std::vector<std::pair<float, float> > &right_arm_joint_limits) const;

protected:
    void setPelvisFrame(const std::string &value);

    void setTorsoFrame(const std::string &value);

    void setLeftArmJointNames(const std::vector<std::string> &left_arm_joint_names);

    void setRightArmJointNames(const std::vector<std::string> &right_arm_joint_names);

    void setLeftArmFrameNames(const std::vector<std::string> &left_arm_frame_names);

    void setRightArmFrameNames(const std::vector<std::string> &right_arm_frame_names);

    void setLeftFootFrameName(const std::string &left_foot_frame_name);

    void setRightFootFrameName(const std::string &getRightFootFrameName);

    void setLeftArmJointLimits(const std::vector<std::pair<float, float> > &left_arm_joint_limits);

    void setRightArmJointLimits(const std::vector<std::pair<float, float> > &right_arm_joint_limits);

    void setLeftPalmFrame(const std::string &value);

    void setRightPalmFrame(const std::string &value);

private:
    RobotDescription(ros::NodeHandle nh, std::string urdf_param="/robot_description");
    static RobotDescription* object;
    urdf::Model model_;
    std::vector<urdf::JointSharedPtr> joints_;
    std::vector<urdf::LinkSharedPtr> links_;
    std::string robot_name_;
    std::string param_left_arm_joint_names_  =  "left_arm_joint_names" ;
    std::string param_right_arm_joint_names_ =  "right_arm_joint_names";
    std::string param_left_foot_frame_name_  =  "left_foot_frame_name" ;
    std::string param_right_foot_frame_name_ =  "right_foot_frame_name";

    std::string PELVIS_TF;
    std::string TORSO_TF;
    std::string R_PALM_TF;
    std::string L_PALM_TF;

    std::vector<std::string> left_arm_joint_names_;
    std::vector<std::string> right_arm_joint_names_;

    std::vector<std::string> left_arm_frame_names_;
    std::vector<std::string> right_arm_frame_names_;

    std::string left_foot_frame_name_;
    std::string right_foot_frame_name_;

    std::vector<std::pair<float, float> > left_arm_joint_limits_;
    std::vector<std::pair<float, float> > right_arm_joint_limits_;


};

#endif // ROBOT_DESCRIPTION_H
