#ifndef VAL_GRIPPER_CONTROL_H
#define VAL_GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <val_common/val_common_defines.h>

enum class GRIPPER_STATE{
    OPEN=0,
    OPEN_THUMB_IN,
    OPEN_THUMB_IN_APPROACH,
    CLOSE,
    HANDLE_HOLD,
    TIGHT_HOLD,
    CUP
};

class gripperControl {

private:

    ros::NodeHandle nh_;
    ros::Publisher leftGripperContPublisher;
    ros::Publisher rightGripperContPublisher;
    //GRIPPER_STATE::OPEN_THUMB_IN
    const std::vector<double> OPEN_THUMB_IN_LEFT_GRIPPER  = {1.3999, 0.2, 0.0, 0.0, 0.0};
    const std::vector<double> OPEN_THUMB_IN_RIGHT_GRIPPER = {1.3999, 0.2, 0.0, 0.0, 0.0};
    //GRIPPER_STATE::OPEN_THUMB_APPROACH
    const std::vector<double> OPEN_THUMB_APPROACH_IN_LEFT_GRIPPER  = {1.3999, -0.3, 0.0, 0.0, 0.0};
    const std::vector<double> OPEN_THUMB_APPROACH_IN_RIGHT_GRIPPER = {1.3999, 0.3, 0.0, 0.0, 0.0};
    //GRIPPER_STATE::CLOSE
    const std::vector<double> CLOSE_LEFT_GRIPPER  = {1.3999, -0.55, -1.1, -0.9, -1.0};
    const std::vector<double> CLOSE_RIGHT_GRIPPER = {1.3999, 0.55, 1.1, 0.9, 1.0};
    //GRIPPER_STATE::CUP
    const std::vector<double> CUP_LEFT_GRIPPER  = {1.3999, -0.2, -0.55, -0.55, -0.55};
    const std::vector<double> CUP_RIGHT_GRIPPER = {1.3999, 0.2, 0.55, 0.55, 0.55};
    //GRIPPER_STATE::HANDLE_HOLD
    const std::vector<double> HANDLE_HOLD_LEFT_GRIPPER  = {2.3, -0.55, -1.1, -0.9, -1.0};
    const std::vector<double> HANDLE_HOLD_RIGHT_GRIPPER = {2.3, 0.55, 1.1, 0.9, 1.0};
    //GRIPPER_STATE::TIGHT_HOLD
    const std::vector<double> TIGHT_HOLD_LEFT_GRIPPER  = {1.6, -1.8, -1.5, -1.6, -1.5};
    const std::vector<double> TIGHT_HOLD_RIGHT_GRIPPER = {1.6, 1.8, 1.5, 1.6, 1.5};

public:
    /**
     * @brief gripperControl class provides ability to control the grippers.
     * @param nh nodehandle to which subscribers and publishers are attached.
     */
    gripperControl(ros::NodeHandle nh);
    ~gripperControl();

    /**
     * @brief controlGripper provides the ability to move the grippers to a desired position.
     * @param side is either LEFT or RIGHT
     * @param gripperData is a vector of size 5.
     */
    void controlGripper(const armSide side, const std::vector<double> gripperData);

    void controlGripper(const armSide side, GRIPPER_STATE state=GRIPPER_STATE::OPEN);

    /**
     * @brief closeGripper closes the gripper completely.
     * @param side is either LEFT or RIGHT
     */
    void closeGripper(const armSide side);

    /**
     * @brief openGripper opens the gripper completely
     * @param side is either LEFT or RIGHT
     */
    void openGripper(const armSide side);
};


#endif // VAL_GRIPPER_CONTROL_H
