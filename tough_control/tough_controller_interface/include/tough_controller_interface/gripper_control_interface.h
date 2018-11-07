#ifndef VAL_GRIPPER_CONTROL_H
#define VAL_GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "ihmc_msgs/HandDesiredConfigurationRosMessage.h"
#include <tough_common/robot_description.h>
#include "tough_controller_interface/tough_controller_interface.h"

#define ROBOTIQ_GRIPPER  //need to find a better way to define this


class GripperControlInterface: public ToughControllerInterface {

private:

    ros::Publisher gripperPublisher_;

public:
    enum GRIPPER_MODES{
        BASIC   = ihmc_msgs::HandDesiredConfigurationRosMessage::BASIC_GRIP,
        PINCH   = ihmc_msgs::HandDesiredConfigurationRosMessage::PINCH_GRIP,
        WIDE    = ihmc_msgs::HandDesiredConfigurationRosMessage::WIDE_GRIP,
        SCISSOR = ihmc_msgs::HandDesiredConfigurationRosMessage::SCISSOR_GRIP,
        RESET   = ihmc_msgs::HandDesiredConfigurationRosMessage::RESET
    };
    /**
     * @brief gripperControl class provides ability to control the grippers.
     * @param nh nodehandle to which subscribers and publishers are attached.
     */
    GripperControlInterface(ros::NodeHandle nh);
    ~GripperControlInterface();

    /**
     * @brief controlGripper provides the ability to move the grippers to a desired position.
     * @param side is either LEFT or RIGHT
     * @param configuration is an enum value from ihmc_msgs::HandDesiredConfigurationRosMessage.
     */
    void controlGripper(const RobotSide side, int configuration=ihmc_msgs::HandDesiredConfigurationRosMessage::BASIC_GRIP);

    /**
     * @brief closeGripper closes the gripper completely.
     * @param side is either LEFT or RIGHT
     */
    void closeGripper(const RobotSide side);

    /**
     * @brief openGripper opens the gripper completely
     * @param side is either LEFT or RIGHT
     */
    void openGripper(const RobotSide side);

#ifdef ROBOTIQ_GRIPPER
    // these modes are only for robotiq grippers

    void setMode(const RobotSide side, const GRIPPER_MODES mode);

    void resetGripper(const RobotSide side);

    void openThumb(const RobotSide side);

    void closeThumb(const RobotSide side);

    void openFingers(const RobotSide side);

    void closeFingers(const RobotSide side);

    void crush(const RobotSide side);

#endif

    virtual bool getJointSpaceState(std::vector<double> &joints, RobotSide side) override;

    virtual bool getTaskSpaceState(geometry_msgs::Pose &pose, RobotSide side, std::string fixedFrame=TOUGH_COMMON_NAMES::WORLD_TF) override;
};


#endif // VAL_GRIPPER_CONTROL_H
