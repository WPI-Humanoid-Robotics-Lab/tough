#ifndef VAL_GRIPPER_CONTROL_H
#define VAL_GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "ihmc_msgs/HandDesiredConfigurationRosMessage.h"
#include <tough_common/robot_description.h>
#include "tough_controller_interface/tough_controller_interface.h"
#include <map>


//Note: HOOK Mode doesn't work. Tested on actual robotiq gripper

class GripperControlInterface: public ToughControllerInterface {

private:

    ros::Publisher gripperPublisher_;

public:
    enum GRIPPER_MODES{
        BASIC   = ihmc_msgs::HandDesiredConfigurationRosMessage::BASIC_GRIP,
        PINCH   = ihmc_msgs::HandDesiredConfigurationRosMessage::PINCH_GRIP,
        WIDE    = ihmc_msgs::HandDesiredConfigurationRosMessage::WIDE_GRIP,
        SCISSOR = ihmc_msgs::HandDesiredConfigurationRosMessage::SCISSOR_GRIP,
        HOOK    = ihmc_msgs::HandDesiredConfigurationRosMessage::HOOK,
        RESET   = ihmc_msgs::HandDesiredConfigurationRosMessage::RESET
    };

    std::map<GRIPPER_MODES,std::string> GRIPPER_MODE_NAMES;
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

    void setMode(const RobotSide side, const GRIPPER_MODES mode);

    void resetGripper(const RobotSide side);

    void openThumb(const RobotSide side);

    void closeThumb(const RobotSide side);

    void openFingers(const RobotSide side);

    void closeFingers(const RobotSide side);

    void crush(const RobotSide side);

    std::string getModeName(const GRIPPER_MODES mode);



    virtual bool getJointSpaceState(std::vector<double> &joints, RobotSide side) override;

    virtual bool getTaskSpaceState(geometry_msgs::Pose &pose, RobotSide side, std::string fixedFrame=TOUGH_COMMON_NAMES::WORLD_TF) override;
};


#endif // VAL_GRIPPER_CONTROL_H
