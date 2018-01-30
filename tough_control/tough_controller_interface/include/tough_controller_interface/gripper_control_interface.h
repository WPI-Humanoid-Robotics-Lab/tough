#ifndef VAL_GRIPPER_CONTROL_H
#define VAL_GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "ihmc_msgs/HandDesiredConfigurationRosMessage.h"
#include <tough_common/robot_description.h>
#include "tough_controller_interface/tough_controller_interface.h"


class GripperControlInterface: public ToughControllerInterface {

private:

    ros::Publisher gripperPublisher_;

public:
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
};


#endif // VAL_GRIPPER_CONTROL_H
