#ifndef VAL_GRIPPER_CONTROL_H
#define VAL_GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <val_common/val_common_defines.h>


class gripperControl {

private:

    ros::NodeHandle nh_;
    ros::Publisher leftGripperContPublisher;
    ros::Publisher rightGripperContPublisher;
    const std::vector<double> CLOSE_LEFT_GRIPPER = {1.4, -0.55, -1.1, -0.9, -1.0};
    const std::vector<double> CLOSE_RIGHT_GRIPPER = {1.4, 0.55, 1.1, 0.9, 1.0};

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
