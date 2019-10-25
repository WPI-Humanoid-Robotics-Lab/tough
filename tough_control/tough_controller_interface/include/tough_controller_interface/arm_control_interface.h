#ifndef ARM_CONTROL_INTERFACE_H
#define ARM_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <ihmc_msgs/ArmTrajectoryRosMessage.h>
#include <ihmc_msgs/ArmDesiredAccelerationsRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <ihmc_msgs/HandDesiredConfigurationRosMessage.h>
#include <ihmc_msgs/HandTrajectoryRosMessage.h>
#include <ihmc_msgs/SE3TrajectoryPointRosMessage.h>
#include <ihmc_msgs/GoHomeRosMessage.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_control_interface.h"

/**
 * @brief The ArmControlInterface class provides ability to move arms of humanoid robots supported by
 * open-humanoids-software.
 */
class ArmControlInterface : public ToughControlInterface
{
public:
  /**
   * @brief ArmControlInterface class provides ability to move arms of humanoid robots supported by
   * open-humanoids-software.
   * @param nh   nodehandle to which subscribers and publishers are attached.
   */
  ArmControlInterface(ros::NodeHandle nh);
  ~ArmControlInterface();

  /**
   * @brief The armJointData struct is a structure that can store details required to generate a ros message for
   * controlling arm.
   *
   * RobotSide can be either RIGHT or LEFT.
   *
   * arm_pose is a vector of float of size 7 that stores joint angels of all 7 joints in the arm.
   *
   * Time is the relative time for executing the trajectory but it increments for every
   * additional trajectory point. For example: if a trajectory needs to be in
   * pose 1 at 2sec, pose 2 at 5sec, then create 2 objects of this struct one for pose 1 and other for pose 2. pose1
   * object will have time=2 and pose2 will have time=5.
   */
  struct ArmJointData
  {
    RobotSide side;
    std::vector<double> arm_pose;
    float time;
  };

  /**
   * @brief The armTaskSpaceData struct is a structure that can store details required to generate a ros message for
   * controlling the hand trajectory in task space.
   *
   * side can be either RIGHT or LEFT.
   * pose is a Pose in task space (world frame) that the hand should move to.
   * time is the total execution time of the trajectory.
   */
  struct ArmTaskSpaceData
  {
    RobotSide side;
    geometry_msgs::Pose pose;
    float time;
  };

  /**
   * @brief moveToDefaultPose Moves the robot arm to default position
   *
   * @param side  Side of the robot. It can be RIGHT or LEFT.
   */
  void moveToDefaultPose(const RobotSide side, const float time = 2.0f);

  /**
   * @brief moveToZeroPose Moves the robot arm to zero position.
   *
   * @param side  Side of the robot. It can be RIGHT or LEFT.
   */
  void moveToZeroPose(const RobotSide side, const float time = 2.0f);

  /**
   * @brief moveArmJoints Moves arm joints to given joint angles. All angles in radians.
   *
   * @param side          Side of the robot. It can be RIGHT or LEFT.
   * @param arm_pose      A vector that stores a vector with 7 values one for each joint. Number of values in the vector
   *                      are the number of trajectory points.
   * @param time          Total time to execute the trajectory. each trajectory point is equally spaced in time.
   */
  bool moveArmJoints(const RobotSide side, const std::vector<std::vector<double> >& arm_pose, const float time);

  /**
   * @brief generateArmMessage Generates ros message to be sent to the arm, but does not publish anything.
   *
   * @param side          Side of the robot. It can be RIGHT or LEFT.
   * @param arm_pose      A vector that stores a vector with 7 values one for each joint. Number of values in the vector
   * are the number of trajectory points.
   * @param time          Total time to execute the trajectory. each trajectory point is equally spaced in time.
   * @param msg           The message is generated in this reference.
   *
   * @return
   */
  bool generateArmMessage(const RobotSide side, const std::vector<std::vector<double> >& arm_pose, const float time,
                          ihmc_msgs::ArmTrajectoryRosMessage& msg);

  /**
   * @brief generateArmMessage Generates ros message to be sent to the arm, but does not publish anything.
   *
   * @param side          Side of the robot. It can be RIGHT or LEFT.
   * @param arm_trajectory A vector of OneDoFJointTrajectoryRosMessage for the trajectory
   * @param msg           The message is generated in this reference.
   *
   * @return
   */
  void generateArmMessage(const RobotSide side,
                          const std::vector<ihmc_msgs::OneDoFJointTrajectoryRosMessage>& arm_trajectory,
                          ihmc_msgs::ArmTrajectoryRosMessage& msg);

  /**
   * @brief generateArmMessage Generates ros message to be sent to the arm, but does not publish anything.
   *
   * @param side                  Side of the robot. It can be RIGHT or LEFT.
   * @param joints_acceleration   A vector that stores a vector with 7 values one for each joint. Number of values in
   * the vector
   *                              are the number of trajectory points.
   * @param msg                   The message is generated in this reference.
   *
   */
  bool generateArmMessage(const RobotSide side, const std::vector<double>& joints_acceleration,
                          ihmc_msgs::ArmDesiredAccelerationsRosMessage& msg);

  /**
   * @brief moveArmJoints Moves arm joints to given joint angles. All angles in radians.
   *
   * @param arm_data      A vector of armJointData struct. This allows customization of individual trajectory points.
   *
   * For example,
   * each point can have different execution times.
   *
   * @return true         - When motion is executed
   * @return false
   */
  bool moveArmJoints(const std::vector<ArmJointData>& arm_data);

  /**
   * @brief moveArmMessage    Publishes a given ros message of ihmc_msgs::ArmTrajectoryRosMessage format to the robot.
   *
   * @param msg               message to be sent to the robot.
   */
  void moveArmMessage(const ihmc_msgs::ArmTrajectoryRosMessage& msg);

  /**
   * @brief getnumArmJoints Gives back the number of arm joints for the robot
   *
   * @return
   */
  int getnumArmJoints() const;

  /**
   * @brief moveArmInTaskSpaceMessage Moves the arm to a given point in task space (world frame)
   *
   * @param side	Side of the robot. It can be RIGHT or LEFT.
   * @param point	The point in task space to move the arm to.
   */
  //    void moveArmInTaskSpaceMessage(const RobotSide side, const ihmc_msgs::SE3TrajectoryPointRosMessage &point, int
  //    baseForControl=ihmc_msgs::FrameInformationRosMessage::CHEST_FRAME);
  void moveArmInTaskSpaceMessage(const RobotSide side, const ihmc_msgs::SE3TrajectoryPointRosMessage& point,
                                 int baseForControl = TOUGH_COMMON_NAMES::PELVIS_ZUP_FRAME_HASH);

  /**
   * @brief moveArmInTaskSpace  Moves the arm to a give pose in task space (world frame)
   *
   * @param side  Side of the robot. It can be RIGHT or LEFT.
   * @param pose  The pose in task space to move the arm to.
   * @param time  Total time to execute the trajectory.
   * @param baseForControl FrameHash in which the pose is defined
   */
  void moveArmInTaskSpace(const RobotSide side, const geometry_msgs::Pose& pose, const float time,
                          int baseForControl = TOUGH_COMMON_NAMES::PELVIS_ZUP_FRAME_HASH);

  /**
   * @brief moveArmInTaskSpace  Moves the arm(s) to the given position in task space (world frame).
   *
   * @param arm_data A vector of armTaskSpaceData struct.
   */
  //    void moveArmInTaskSpace(std::vector<armTaskSpaceData> &arm_data, int
  //    baseForControl=ihmc_msgs::FrameInformationRosMessage::CHEST_FRAME);
  void moveArmInTaskSpace(const std::vector<ArmTaskSpaceData>& arm_data,
                          const int baseForControl = TOUGH_COMMON_NAMES::PELVIS_ZUP_FRAME_HASH);

  /**
   * @brief moveArmTrajectory Moves the arm to follow a particular trajectory plan
   *
   * @param side              Side of the robot. It can be RIGHT or LEFT.
   * @param traj              Trajectory in the form of trajectory_msgs::JointTrajectory
   */
  void moveArmTrajectory(const RobotSide side, const trajectory_msgs::JointTrajectory& traj);

  /**
   * @brief moveArmJointsAcceleration moves the arm joints with the desired joint accelration.
   * This message gives the user the option to bypass IHMC feedback controllers for the arm joints by sending desired
   arm joint accelerations.
     One needs experience in control when activating the bypass as it can result in unexpected behaviors for
   unreasonable accelerations.

   * @param side                Side of the Robot. it can be LEFT or RIGHT
   * @param accelration_vector  vector of the values for the acceleration of the arm joints.
   *
   * @return true               When the trajectory is executed
   * @return false
   */
  bool moveArmJointsAcceleration(const RobotSide side, const std::vector<double>& accelration_vector);

  /**
   * @brief nudgeArm Nudges the Arm in the desired direction by a given nudge step with respect
   *            to the pelvis frame of the robot.
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param drct              Which side we want to nudge. UP, DOWN, LEFT, RIGHT, FRONT or BACK
   * @param nudgeStep         The step length to nudge. Default is 5cm (~6/32")
   *
   * @return true             When the trajectory is executed
   * @return false
   */
  bool nudgeArm(const RobotSide side, const direction drct, const float nudgeStep = 0.05);

  /**
   * @brief nudgeArmLocal     Nudges the Arm in the desired direction by a given nudge step with
   *              respect to the local frame, which is the palm frame of the robot.
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param drct              Which side we want to nudge. UP, DOWN, LEFT, RIGHT, FRONT or BACK
   * @param nudgeStep         The step length to nudge. Default is 5cm (~6/32")
   *
   * @return true             When the trajectory is executed
   * @return false
   */
  bool nudgeArmLocal(const RobotSide side, const direction drct, const float nudgeStep = 0.05);

  /**
   * @brief Generates the vector of the ArmTaskSpaceData. This does not publish anything, only
   * sets the ArmTaskSpaceData, for the desired pose.
   *
   * @param input_poses       Input poses for the data generation
   * @param input_side        Side of the Robot. it can be LEFT or RIGHT
   * @param desired_time      Desired time for the execution of the trajectory
   * @param arm_data_vector   [output]
   * @return true             When successful
   * @return false
   */
  bool generate_task_space_data(const std::vector<geometry_msgs::PoseStamped>& input_poses, const RobotSide input_side,
                                const float desired_time,
                                std::vector<ArmControlInterface::ArmTaskSpaceData>& arm_data_vector);

  /**
   * @brief Moves the specific joint of arm of the robot.
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param jointNumber       The jointNumber of the joint to be moved.
   * @param targetAngle       Desired angle for the motion
   * @param time              Time of execution of the trajectory
   * @return true             When successfull
   * @return false
   */
  bool moveArmJoint(const RobotSide side, const int jointNumber, const float targetAngle, const float time = 2.0f);

  /**
   * @brief Get the current positions of all joints of the side of the arm.
   * Ordering is based on the order in the JointNames vector. The order for the Joints' Names, Numbers,
   * Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param joints            [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @return true             when successful
   * @return false
   */
  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  /**
   * @brief Get the Current Pose of the End effector frame with respect to the fixedFrame frame
   *
   * @param pose              [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param fixedFrame        Reference frame for the state query
   * @return true             When successful
   * @return false
   */
  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                                 std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;

  /**
   * @brief This will append a joint trajectory point
   *
   * @param msg               msg is the reference of the current msg where point is to be appended.
   * @param time              time is the time between last and current joint trajectory waypoint
   * @param pos               pos is the joint position vector (size would be 7 if there are 7 joints in the arm)
   */
  void appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage& msg, const float time, const std::vector<double>& pos);

  /**
   * @brief Sets up the msg for usage. This does not either enters data into the msg, or execute any
   * type of message.
   *
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param msg               [output]
   * @return true             when successful
   * @return false
   */
  bool setupArmMessage(const RobotSide side, ihmc_msgs::ArmTrajectoryRosMessage& msg);

private:
  const std::vector<double> ZERO_POSE;
  int NUM_ARM_JOINTS;
  std::vector<std::pair<double, double> > joint_limits_left_;
  std::vector<std::pair<double, double> > joint_limits_right_;

  ros::Publisher armTrajectoryPublisher;
  ros::Publisher armJointAccelerationPublisher;
  ros::Publisher handTrajectoryPublisher;
  ros::Publisher taskSpaceTrajectoryPublisher;
  ros::Publisher homePositionPublisher;
  ros::Publisher markerPub_;
  ros::Subscriber armTrajectorySubscriber;

  void poseToSE3TrajectoryPoint(const geometry_msgs::Pose& pose, ihmc_msgs::SE3TrajectoryPointRosMessage& point);
  void appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage& msg,
                             const trajectory_msgs::JointTrajectoryPoint& point);
};

#endif  // ARM_CONTROL_INTERFACE_H
