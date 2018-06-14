#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <urdf/model.h>
#include "tough_common/tough_common_names.h"

// defines for arms
enum RobotSide
{
    LEFT = 0,
    RIGHT
};

enum class direction{
    LEFT = 0,  //Positive Y 0
    RIGHT,     //Negative Y 1
    UP,        //Positive Z 2
    DOWN,      //Negative Z 3
    FRONT,     //Positive X 4
    BACK       //Negative X 5
};

enum class EE_LOADING{
    LOAD=0,
    UNLOAD
};



class RobotDescription
{
public:

    static RobotDescription* getRobotDescription(ros::NodeHandle nh, std::string urdf_param="/robot_description");

    //disable assign and copy

    RobotDescription(RobotDescription const&)   = delete;
    void operator=(RobotDescription const&)     = delete;

    const std::string getPelvisFrame() const;

    const std::string getWorldFrame() const;

    const std::string getTorsoFrame() const;

    const std::string getLeftFootFrameName() const;

    const std::string getRightFootFrameName() const;

    const std::string getLeftPalmFrame() const;

    const std::string getRightPalmFrame() const;

    const std::string getRightEEFrame() const;

    const std::string getLeftEEFrame() const;

    const std::string getRobotName() const;

    void getLeftArmJointNames(std::vector<std::string> &left_arm_joint_names) const;

    void getRightArmJointNames(std::vector<std::string> &right_arm_joint_names) const;

    void getLeftArmFrameNames(std::vector<std::string> &left_arm_frame_names) const;

    void getRightArmFrameNames(std::vector<std::string>  &right_arm_frame_names) const;

    void getLeftArmJointLimits(std::vector<std::pair<float, float> > &left_arm_joint_limits) const;

    void getRightArmJointLimits(std::vector<std::pair<float, float> > &right_arm_joint_limits) const;

    int getNumberOfNeckJoints() const;


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

    void setNumberOfNeckJoints(int numberOfNeckJoints);

private:
    RobotDescription(ros::NodeHandle nh, std::string urdf_param="/robot_description");
    ~RobotDescription();
    static RobotDescription* object;
    urdf::Model model_;
    std::vector<urdf::JointSharedPtr> joints_;
    std::vector<urdf::LinkSharedPtr> links_;
    std::string robot_name_;
    std::string param_left_arm_joint_names_  ;
    std::string param_right_arm_joint_names_ ;
    std::string param_left_foot_frame_name_  ;
    std::string param_right_foot_frame_name_ ;

    std::string PELVIS_TF;
    std::string TORSO_TF;

    std::string R_PALM_TF;
    std::string L_PALM_TF;

    std::string R_END_EFFECTOR_TF;
    std::string L_END_EFFECTOR_TF;

    std::vector<std::string> left_arm_joint_names_;
    std::vector<std::string> right_arm_joint_names_;

    std::vector<std::string> left_arm_frame_names_;
    std::vector<std::string> right_arm_frame_names_;

    std::string left_foot_frame_name_;
    std::string right_foot_frame_name_;

    std::vector<std::pair<float, float> > left_arm_joint_limits_;
    std::vector<std::pair<float, float> > right_arm_joint_limits_;

    int number_of_neck_joints_;
};

#endif // ROBOT_DESCRIPTION_H
