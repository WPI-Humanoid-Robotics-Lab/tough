#ifndef VAL_PELVIS_NAVIGATION_H
#define VAL_PELVIS_NAVIGATION_H

#include <ihmc_msgs/PelvisHeightTrajectoryRosMessage.h>
#include <ihmc_msgs/GoHomeRosMessage.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tough_common/robot_state.h"
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_control_interface.h"

/**
 * @brief The PelvisControlInterface class provides ability to control pelvis of humanoid robots supported by
 * open-humanoids-software.
 *
 */
class PelvisControlInterface : public ToughControlInterface
{
private:
  ros::Publisher pelvisHeightPublisher_;
  ros::Publisher homePositionPublisher_;

public:
  /**
   * @brief The PelvisControlInterface class provides ability to control pelvis of humanoid robots supported by
   * open-humanoids-software.
   *
   * @param nh   nodehandle to which subscribers and publishers are attached.
   */
  PelvisControlInterface(ros::NodeHandle nh);
  ~PelvisControlInterface();
  
  /**
   * @brief Controls the pelvis hight of the robot
   * 
   * @param height                Desired hight of the pelvis.
   * @param duration              Time of execution for the motion
   */
  void controlPelvisHeight(float height, float duration = 2.0f);

  /**
   * @brief Executes the PelvisHeightTrajectoryRosMessage to control the pelvis
   *
   * @param msg                   Executes the IHMC message to control the pelvis.
   */
  void publishPelvisMessage(const ihmc_msgs::PelvisHeightTrajectoryRosMessage& msg) const;

  /**
   * @brief Resets the Pelvis pose and height
   *
   * @param time                  Executes the IHMC message to control the pelvis.
   */
  void resetPose(float time = 1.0f);

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

#endif
