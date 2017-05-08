#ifndef BUTTON_PRESS_H
#define BUTTON_PRESS_H
#include <val_task2/button_detector.h>
#include <val_control/val_arm_navigation.h>
#include <val_control/val_gripper_control.h>


class button_press
{
    armTrajectory armTraj_;
    ros::NodeHandle nh_;
    button_detector bd_;
    tf::TransformListener listener_;
    gripperControl gripper_;
    RobotStateInformer *current_state_;
    geometry_msgs::QuaternionStamped leftHandOrientation_ ;
    geometry_msgs::QuaternionStamped rightHandOrientation_;


    /*Top Grip*/
    const std::vector<float> leftShoulderSeed_ = {-0.23, -0.72, 0.65, -1.51, 2.77, 0.0, 0.0};
    /*Side Grip*/
 //   const std::vector<float> leftShoulderSeed_ = {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
    const std::vector<float> rightShoulderSeed_ = {-0.23, 0.72, 0.65, 1.51, 2.77, 0.0, 0.0};

    const std::vector<double> leftGripperSeed_ = {0.0, 0.0, -1.1, -0.9, -1.0};

    const std::vector<double> rightGripperSeed_ = {0.0, 0.0, 1.1, 0.9, 1.0};


public:

    void grasp_button(const armSide side, geometry_msgs::Point &goal, float executionTime=2.0f);

    geometry_msgs::QuaternionStamped leftHandOrientation() const;
    void setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation);

    geometry_msgs::QuaternionStamped rightHandOrientation() const;
    void setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation);

    button_press(ros::NodeHandle&);
    ~button_press();

};

#endif // BUTTON_PRESS_H
