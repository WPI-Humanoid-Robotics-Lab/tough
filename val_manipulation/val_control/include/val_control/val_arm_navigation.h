#ifndef VAL_ARM_NAVIGATION_H
#define VAL_ARM_NAVIGATION_H

#include <ros/ros.h>
#include <ihmc_msgs/ArmTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <val_common/val_common_defines.h>

/**
 * @brief The armTrajectory class provides ability to move arms of valkyrie. Current implementation provides joint level without collision detection.
 * @todo  Add taskspace control. Add collision detection and avoidance while generating trajectories.
 */
class armTrajectory {

public:
    /**
     * @brief armTrajectory class provides ability to move arms of valkyrie. Current implementation provides joint level without collision detection.
     * @param nh   nodehandle to which subscribers and publishers are attached.
     */
    armTrajectory(ros::NodeHandle nh);
    ~armTrajectory();

    /**
     * @brief The armJointData struct is a structure that can store details required to generate a ros message for controlling arm. Side can be
     * either RIGHT or LEFT. arm_pose is a vector of float of size 7 that stores joint angels of all 7 joints in the arm. time is the relative
     * time for executing the trajectory but it increments for every additional trajectory point. For example: if a trajectory needs to be in
     * pose 1 at 2sec, pose 2 at 5sec, then create 2 objects of this struct one for pose 1 and other for pose 2. pose1 object will have time=2
     * and pose2 will have time=5. This means that executing trajectory to reach pose 2 will take 5-2=3 sec.
     */
    struct armJointData {
        armSide side;
        std::vector<float> arm_pose;
        float time;
    };

    /**
     * @brief moveToDefaultPose Moves the robot arm to default position
     * @param side  Side of robot. It can be RIGHT or LEFT.
     */
    void moveToDefaultPose(armSide side);

    /**
     * @brief moveToZeroPose Moves the robot arm to zero position.
     * @param side  Side of the robot. It can be RIGHT or LEFT.
     */
    void moveToZeroPose(armSide side);

    /**
     * @brief moveArmJoints Moves arm joints to given joint angles. All angles in radians.
     * @param side          Side of the robot. It can be RIGHT or LEFT.
     * @param arm_pose      A vector that stores a vector with 7 values one for each joint. Number of values in the vector are the number of trajectory points.
     * @param time          Total time to execute the trajectory. each trajectory point is equally spaced in time.
     */
    void moveArmJoints(const armSide side,const std::vector<std::vector<float> > arm_pose,const float time);

    /**
     * @brief moveArmJoints Moves arm joints to given joint angles. All angles in radians.
     * @param arm_data      A vector of armJointData struct. This allows customization of individual trajectory points. For example,
     * each point can have different execution times.
     */
    void moveArmJoints(std::vector<armJointData> arm_data);

    /**
     * @brief moveArmMessage    Publishes a given ros message of ihmc_msgs::ArmTrajectoryRosMessage format to the robot.
     * @param msg               message to be sent to the robot.
     */
    void moveArmMessage(const ihmc_msgs::ArmTrajectoryRosMessage& msg);


private:

    static int arm_id;

    const std::vector<float> ZERO_POSE;
    const std::vector<float> DEFAULT_POSE;

    ros::NodeHandle nh_;
    ros::Publisher  armTrajectoryPublisher;
    ros::Publisher  handTrajectoryPublisher;
    ros::Subscriber armTrajectorySunscriber;

    void appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos);

};

int armTrajectory::arm_id = -1;
#endif // VAL_ARM_NAVIGATION_H
