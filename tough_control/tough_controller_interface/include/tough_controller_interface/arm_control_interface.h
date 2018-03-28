#ifndef ARM_CONTROL_INTERFACE_H
#define ARM_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <ihmc_msgs/ArmTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <ihmc_msgs/HandTrajectoryRosMessage.h>
#include <ihmc_msgs/SE3TrajectoryPointRosMessage.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_controller_interface.h"

/**
 * @brief The ArmControlInterface class provides ability to move arms of humanoid robots supported by open-humanoids-software.
 */
class ArmControlInterface: public ToughControllerInterface {

public:
    /**
     * @brief ArmControlInterface class provides ability to move arms of humanoid robots supported by open-humanoids-software.
     * @param nh   nodehandle to which subscribers and publishers are attached.
     */
    ArmControlInterface(ros::NodeHandle nh);
    ~ArmControlInterface();

    /**
     * @brief The armJointData struct is a structure that can store details required to generate a ros message for controlling arm. RobotSide can be
     * either RIGHT or LEFT. arm_pose is a vector of float of size 7 that stores joint angels of all 7 joints in the arm. Time is the relative
     * time for executing the trajectory but it increments for every additional trajectory point. For example: if a trajectory needs to be in
     * pose 1 at 2sec, pose 2 at 5sec, then create 2 objects of this struct one for pose 1 and other for pose 2. pose1 object will have time=2
     * and pose2 will have time=5.
     */
    struct ArmJointData {
        RobotSide side;
        std::vector<float> arm_pose;
        float time;
    };

    /**
     * @brief The armTaskSpaceData struct is a structure that can store details required to generate a ros message for controlling the hand trajectory in task space.
     * side can be either RIGHT or LEFT. pose is a Pose in task space (world frame) that the hand should move to. time is the total execution time of the trajectory.
     */
    struct ArmTaskSpaceData {
      RobotSide side;
      geometry_msgs::Pose pose;
      float time;
    };

    /**
     * @brief moveToDefaultPose Moves the robot arm to default position
     * @param side  Side of the robot. It can be RIGHT or LEFT.
     */
    void moveToDefaultPose(RobotSide side, float time=2.0f);

    /**
     * @brief moveToZeroPose Moves the robot arm to zero position.
     * @param side  Side of the robot. It can be RIGHT or LEFT.
     */
    void moveToZeroPose(RobotSide side, float time=2.0f);

    /**
     * @brief moveArmJoints Moves arm joints to given joint angles. All angles in radians.
     * @param side          Side of the robot. It can be RIGHT or LEFT.
     * @param arm_pose      A vector that stores a vector with 7 values one for each joint. Number of values in the vector are the number of trajectory points.
     * @param time          Total time to execute the trajectory. each trajectory point is equally spaced in time.
     */
    bool moveArmJoints(const RobotSide side,const std::vector<std::vector<float> > &arm_pose,const float time);

    /**
     * @brief moveArmJoints Moves arm joints to given joint angles. All angles in radians.
     * @param arm_data      A vector of armJointData struct. This allows customization of individual trajectory points. For example,
     * each point can have different execution times.
     */
    bool moveArmJoints(std::vector<ArmJointData> &arm_data);

    /**
     * @brief moveArmMessage    Publishes a given ros message of ihmc_msgs::ArmTrajectoryRosMessage format to the robot.
     * @param msg               message to be sent to the robot.
     */
    void moveArmMessage(const ihmc_msgs::ArmTrajectoryRosMessage &msg);

    /**
     * @brief getnumArmJoints Gives back the number of arm joints for Valkyrie R5
     * @return
     */
    int getnumArmJoints() const;

    /**
     * @brief closeHand	Closed the hand on the give side of Valkyrie R5.
     * @param side	Side of the robot. It can be RIGHT or LEFT.
     */
    void closeHand(const RobotSide side);

    /**
     * @brief moveArmInTaskSpaceMessage Moves the arm to a given point in task space (world frame)
     * @parm side	Side of the robot. It can be RIGHT or LEFT.
     * @param point	The point in task space to move the arm to.
     */
//    void moveArmInTaskSpaceMessage(const RobotSide side, const ihmc_msgs::SE3TrajectoryPointRosMessage &point, int baseForControl=ihmc_msgs::FrameInformationRosMessage::CHEST_FRAME);
    void moveArmInTaskSpaceMessage(const RobotSide side, const ihmc_msgs::SE3TrajectoryPointRosMessage &point, int baseForControl=0);

    /**
     * @brief moveArmInTaskSpace  Moves the arm to a give pose in task space (world frame)
     * @param side  Side of the robot. It can be RIGHT or LEFT.
     * @param pose  The pose in task space to move the arm to.
     * @param time  Total time to execute the trajectory.
     */
    void moveArmInTaskSpace(const RobotSide side, const geometry_msgs::Pose &pose, const float time);

    /**
     * @brief moveArmInTaskSpace  Moves the arm(s) to the given position in task space (world frame).
     * @param arm_data A vector of armTaskSpaceData struct.
     */
//    void moveArmInTaskSpace(std::vector<armTaskSpaceData> &arm_data, int baseForControl=ihmc_msgs::FrameInformationRosMessage::CHEST_FRAME);
    void moveArmInTaskSpace(std::vector<ArmTaskSpaceData> &arm_data, int baseForControl=-102);

    /**
     * @brief moveArmTrajectory Moves the arm to follow a particular trajectory plan
     * @param side              Side of the robot. It can be RIGHT or LEFT.
     * @param traj              Trajectory in the form of trajectory_msgs::JointTrajectory
     */
    void moveArmTrajectory(const RobotSide side, const trajectory_msgs::JointTrajectory &traj);

    void testPrint();


    /**
     * @brief nudgeArm Nudges the Arm in the desired direction by a given nudge step
     * @param side     Side of the Robot. it can be LEFT or RIGHT
     * @param drct     Which side we want to nudge. UP, DOWN, LEFT, RIGHT, FRONT or BACK
     * @param nudgeStep The step length to nudge. Default is 5cm (~6/32")
     * @return
     */
    bool nudgeArm(const RobotSide side, const direction drct, float nudgeStep = 0.05);

    bool nudgeArmLocal(const RobotSide side, const direction drct, float nudgeStep = 0.05);

    bool generate_task_space_data(const std::vector<geometry_msgs::PoseStamped>& input_poses,const RobotSide input_side,const float desired_time, std::vector<ArmControlInterface::ArmTaskSpaceData> &arm_data_vector);

    bool moveArmJoint(const RobotSide side, int jointNumber, const float targetAngle, float time=2.0f);

    bool nudgeArmLocal(const RobotSide side, float x, float y, float z,geometry_msgs::Pose &pose);
    bool nudgeArmPelvis(const RobotSide side, float x, float y, float z,geometry_msgs::Pose &pose);

private:


    const std::vector<float> ZERO_POSE;
    const std::vector<float> DEFAULT_RIGHT_POSE;
    const std::vector<float> DEFAULT_LEFT_POSE;
    int NUM_ARM_JOINTS;
    std::vector<std::pair<float, float> > joint_limits_left_;
    std::vector<std::pair<float, float> > joint_limits_right_;

    ros::Publisher  armTrajectoryPublisher;
    ros::Publisher  handTrajectoryPublisher;
    ros::Publisher  taskSpaceTrajectoryPublisher;
    ros::Publisher  markerPub_;
    ros::Subscriber armTrajectorySubscriber;

    void poseToSE3TrajectoryPoint(const geometry_msgs::Pose &pose, ihmc_msgs::SE3TrajectoryPointRosMessage &point);
    void appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos);
    void appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, trajectory_msgs::JointTrajectoryPoint point);

};

#endif // ARM_CONTROL_INTERFACE_H
