#ifndef CHEST_CONTROL_INTERFACE_H
#define CHEST_CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <ihmc_msgs/SpineDesiredAccelerationsRosMessage.h>
#include <ihmc_msgs/GoHomeRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_control_interface.h"

/**
 * @brief The ChestControlInterface class provides ability to move chest of humanoid robots supported by
 * open-humanoids-software.
 */
class ChestControlInterface : public ToughControlInterface
{
private:
  ros::Publisher chestTrajPublisher_;
  ros::Publisher homePositionPublisher_;
  ros::Publisher spineAccnPublisher_;
  std::vector<std::string> chestJointNames_;
  std::vector<int> chestJointNumbers_;

public:
  /**
   * @brief The ChestControlInterface class provides ability to move chest of humanoid robots supported by
   * open-humanoids-software.
   *
   * @param nh   nodehandle to which subscribers and publishers are attached.
   */
  ChestControlInterface(ros::NodeHandle nh);
  ~ChestControlInterface();

  /**
   * @brief controls the chest to achieve the roll, pitch and yaw angles. All the angles are in Radians
   *
   * @param roll                      Desired roll angle for the chest (Radians)
   * @param pitch                     Desired pitch angle for the chest (Radians)
   * @param yaw                       Desired yaw angle for the chest (Radians)
   * @param time                      Time of execution for the trajectory.
   * @param execution_mode            Whether to OVERRIDE the ongoing trajectory, or QUEUE
   */
  void controlChest(const float roll, const float pitch, const float yaw, const float time = 1.0f,
                    int execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE);

  /**
   * @brief controls the chest to achieve the desired Quaternion angle.
   *
   * @param quat                      Desired Quaternions for the chest control
   * @param time                      Time of execution for the trajectory.
   * @param execution_mode            Whether to OVERRIDE the ongoing trajectory, or QUEUE
   */
  void controlChest(const geometry_msgs::Quaternion quat, const float time = 1.0f,
                    int execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE);

  /**
   * @brief Executes the chest accelerations given on the robot. DOES NOT CONTROL IT
   * This message gives the user the option to bypass IHMC feedback controllers for the arm joints 
   * by sending desired arm joint accelerations.
   * One needs experience in control when activating the bypass as it can result in unexpected 
   * behaviors for unreasonable accelerations.
   * 
   * @param chest_accelerations      Vector of chest accelerations to be executed.
   * 
   * @return true                    If the motion is executed.
   * @return false                   If the motion is not executed.
   */
  bool executeChestAccelerations(const std::vector<double>& chest_accelerations);

  /**
   * @brief Execute the ChestTrajectoryRosMessage
   *
   * @param msg                       ChestTrajectoryRosMessage to be executed
   */
  void executeMessage(const ihmc_msgs::ChestTrajectoryRosMessage& msg);

  /**
   * @brief Generates the ChestTrajectoryRosMessage from the quaternions. Does not publish or executes anything, only
   * generates the ros message.
   *
   * @param quat                      Desired Quaternions for the chest control
   * @param time                      Time of execution for the trajectory.
   * @param execution_mode            Whether to OVERRIDE the ongoing trajectory, or QUEUE
   * @param msg                       [output]
   */
  void generateMessage(const geometry_msgs::Quaternion& quat, const float time, const int execution_mode,
                       ihmc_msgs::ChestTrajectoryRosMessage& msg);

  /**
   * @brief Generates the ChestTrajectoryRosMessage from the IHMC message. Does not publish or executes anything, only
   * generates the ros message.
   *
   *
   * @param chest_trajectory          Chest trajectory to be executed
   * @param execution_mode            Whether to OVERRIDE the ongoing trajectory, or QUEUE
   * @param msg                       [output]
   */
  void generateMessage(const std::vector<ihmc_msgs::SO3TrajectoryPointRosMessage>& chest_trajectory,
                       const int execution_mode, ihmc_msgs::ChestTrajectoryRosMessage& msg);

  /**
   * @brief Generates the ChestTrajectoryRosMessage from the IHMC message. Does not publish or executes anything, only
   * generates the ros message.
   *
   *
   * @param chest_accelerations       Chest accelerations to be given
   * @param msg                       [output]
   */
  bool generateMessage(const std::vector<double>& chest_accelerations,
                       ihmc_msgs::SpineDesiredAccelerationsRosMessage& msg);

  /**
   * @brief Sets up the msg for usage. This does not either enters data into the msg, or execute any
   * type of message.
   *
   * @param msg                       [output]
   * @param mode                      Whether to OVERRIDE the ongoing trajectory, or QUEUE
   * @param frame_hash                The reference frame for the chest control.
   */
  void setupFrameAndMode(ihmc_msgs::ChestTrajectoryRosMessage& msg,
                         const int mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE,
                         const int frame_hash = TOUGH_COMMON_NAMES::PELVIS_ZUP_FRAME_HASH);

  /**
   * @brief This will append a joint trajectory point
   *
   * @param q_in                      Desired Quaternions for the chest control
   * @param msg                       [output]
   * @param time                      Time of execution for the trajectory.
   */
  void appendChestTrajectoryPoint(const geometry_msgs::Quaternion q_in, ihmc_msgs::ChestTrajectoryRosMessage& msg,
                                  const double time = 2.0f);

  /**
   * @brief Get the Chest Orientation for the desired Quaternion
   *
   * @param orientation               Desired Quaternions for the chest control
   */
  void getChestOrientation(geometry_msgs::Quaternion& orientation);

  /**
   * @brief Resets the chest orientation to default zero pose.
   *
   * @param time                      Time of execution for the trajectory.
   */
  void resetPose(float time = 2.0f);

  /**
   * @brief Get the current positions of all joints of the side of the chest.
   * Ordering is based on the order in the JointNames vector. The order for the Joints' Names, Numbers,
   * Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param joints                    [output]
   * @return true                     When Successful
   * @return false
   */
  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side) override;

  /**
   * @brief Get the current positions of all joints of the side of the chest.
   * Ordering is based on the order in the JointNames vector. The order for the Joints' Names, Numbers,
   * Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param pose              [output]
   * @param side              Side of the Robot. it can be LEFT or RIGHT
   * @param fixedFrame        Reference frame for the state query
   * @return true             When successful
   * @return false
   */
  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side,
                                 std::string fixedFrame = TOUGH_COMMON_NAMES::WORLD_TF) override;
};

#endif  // CHEST_CONTROL_INTERFACE_H
