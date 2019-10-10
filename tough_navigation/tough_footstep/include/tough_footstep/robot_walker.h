#ifndef ROBOT_WALKER_H
#define ROBOT_WALKER_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include "ihmc_msgs/WalkingStatusRosMessage.h"
#include "ihmc_msgs/FootTrajectoryRosMessage.h"
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"
#include "ihmc_msgs/AbortWalkingRosMessage.h"
#include "ihmc_msgs/PauseWalkingRosMessage.h"
#include "ihmc_msgs/EndEffectorLoadBearingRosMessage.h"
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tough_common/robot_state.h>
#include "tough_common/robot_description.h"
#include "tough_common/tough_common_names.h"
/**
 * @brief The RobotWalker class This class handles all the locomotion commands to the robot.
 */
class RobotWalker
{
public:
  static int id;

  /**
   * @brief RobotWalker::RobotWalker This class handles all the locomotion commands.
   * @param nh    ros nodehandle
   * @param InTransferTime    Time the controllers should take to transfer weight of the robot from one leg to another
   * @param InSwingTime       Time the controllers should take to swing a leg while walking
   * @param InMode            ExecutionMode can be set to OVERRIDE or QUEUE
   * @param swingHeight       Height to which a swing foot should be raised while walking
   */
  RobotWalker(ros::NodeHandle nh, const double inTransferTime = 1.5, const double inSwingTime = 1.5,
              const int inMode = 0, const double swing_height_ = 0.2);
  ~RobotWalker();

  /**
   * @brief walkToGoal walks to a given 2D point in a map. needs a map either from map server or from octomap server and
   * the footstep_planner service
   * @param goal  pose2d message giving position and orientation of goal point.
   * @return true if footstep planning is successful else false
   */
  bool walkToGoal(const geometry_msgs::Pose2D &goal, const bool waitForSteps = true);

  /**
   * @brief stepAtPose Steps at the given goal location in single step. Both position and orientation is used from the
   * provided goal.
   *
   * @param goal  position and orientation of the desired footstep
   * @param side  specifies the foot to be used for stepping
   * @param waitForSteps  set this to true for a blocking call, otherwise false.
   */
  void stepAtPose(const geometry_msgs::Pose &goal, const RobotSide side, const bool waitForSteps = false,
                  const bool queue = false, const std::string refFrame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief walkNSteps Makes the robot walk given number of steps. The offsets are in world frame.
   * @param n          Number of steps to walk
   * @param x_offset   distance to travel forward in half stride. First step is half the stride length as both the
   * feet are assumed to be together.
   * @param y_offset   distance to travel sideways in half a stride length. First step is half the stride length as
   * both the feet are assumed to be together.
   * @param startLeg   leg to be used to start walking. It can be RIGHT or LEFT
   * @return
   */
  bool walkNSteps(const int n, const float x_offset, float y_offset = 0.0f, const RobotSide startLeg = RIGHT,
                  const bool waitForSteps = true);
  /**
   * @brief walkNStepsWRTPelvis Makes the robot walk given number of steps. The offsets are in pelvis frame.
   * @param n          Number of steps to walk
   * @param x_offset   distance to travel forward in half stride. First step is half the stride length as both the
   * feet are assumed to be together.
   * @param y_offset   distance to travel sideways in half a stride length. First step is half the stride length as
   * both the feet are assumed to be together.
   * @param startLeg   leg to be used to start walking. It can be RIGHT or LEFT
   * @return
   */
  bool walkNStepsWRTPelvis(const int n, const float x_offset, float y_offset = 0.0f, const RobotSide startLeg = RIGHT,
                           const bool waitForSteps = true);

  /**
   * @brief walkPreComputedSteps If the steps to be sent to the robot are not identical, use this function to send steps
   * that are precomputed.
   * @param x_offset  is a vector of float. Each value represents offset in x direction of individual step
   * @param y_offset  is a vector of float with size same as that of x_offset. Each value represents offset in y
   * direction of individual step
   * @param startleg  leg to be used to start walking. It can be RIGHT or LEFT
   * @return
   */
  bool walkPreComputedSteps(const std::vector<float> &x_offset, const std::vector<float> &y_offset,
                            const RobotSide startleg);

  /**
   * @brief walkGivenSteps This function publishes a given list of ros messages of type
   * ihmc_msgs::FootstepDataListRosMessage to the robot.
   * @param list           List of steps in ihmc_msgs::FootstepDataListRosMessage format.
   * @return
   */
  bool walkGivenSteps(const ihmc_msgs::FootstepDataListRosMessage &list, const bool waitForSteps = true);

  /**
   * @brief setWalkParms      Set the values of walking parameters
   * @param InTransferTime    transfer_time is the time required for the robot to switch its weight from one to other
   * while walking.
   * @param InSwingTime       swing_time is the time required for the robot to swing its leg to the given step size.
   * @param InMode            execution mode defines if steps are to be queued with previous steps or override and start
   * a walking message. Only Override is supported in this version.
   * @todo create separate messages for each of the parameters.
   */
  inline void setWalkParams(const float InTransferTime, const float InSwingTime, const int InMode)
  {
    this->transfer_time_ = InTransferTime;
    this->swing_time_ = InSwingTime;
    this->execution_mode_ = InMode;
  }

  /**
   * @brief getSwingHeight fetch the swing height used for steps.
   * @return returns the swing_height of the current object.
   */
  double getSwingHeight() const;

  /**
   * @brief setSwing_height Sets swing_height for walking.
   * @param value           Value is the swing_height that determines how high a feet should be lifted while walking in
   * meters. It should be between 0.1 and 0.25     *
   */
  inline void setSwingHeight(const double value)
  {
    swing_height_ = value;
  }

  /**
   * @brief turn contains precomputed footsteps to make a left or right 90 degree turn
   * @param side left- anticlockwise, right - clockwise.
   * DO NOT USE. ROBOT MIGHT FALL.
   * @return
   */
  bool turn(const RobotSide side);

  /**
   * @brief walkLocalPreComputedSteps walks predefined steps which could have varying step length and step widths. This
   * is defined wrt Pelvis frame.
   * @param xOffset  Is a vector of float. Each value represents offset in x direction of individual step.
   *                 you can define a set a predefined step length offsets.
   * @param yOffset  Is a vector of float with size same as that of x_offset. Each value represents offset in y
   * direction of individual step
   *                 you can define a set a predefined step widths offsets.
   * @param startleg Leg to be used to start walking. It can be RIGHT or LEFT
   * @return
   */
  bool walkLocalPreComputedSteps(const std::vector<float> &xOffset, const std::vector<float> &yOffset,
                                 const RobotSide startLeg);

  /**
   * @brief curlLeg would curl the leg behind with a defined radius. it is similar to the flamingo position.
   * @param side LEFT/RIGHT
   * @param radius is the radius of this backward curled trajectory
   * @return
   */
  bool curlLeg(const RobotSide side, const float radius, const float time = 3.0f);

  /**
   * @brief alignFeet Checks if the feet are aligned, if not, takes one step to align their position and orientation
   *
   * @param side is the leg used for stepping
   */
  void alignFeet(const RobotSide side = RobotSide::LEFT);
  /**
   * @brief placeLeg is used when the swing leg is in a arbitrary lifted position and has to be placed within a z-offset
   * with the support leg.
   *        it can be thought as a way to bring the robot from the flamingo position to a normal position with a
   * z-offset.
   * @param side LEFT/RIGHT
   * @param offset is the offset in the z-axis
   * @return
   */
  bool placeLeg(const RobotSide side, const float offset = 0.1f, const float time = 2.0f);

  /**
   * @brief nudgeFoot nudges the foot forward or backward
   * @param side LEFT/RIGHT
   * @param distance is the offset distance
   * @return
   */
  bool nudgeFoot(const RobotSide side, const float x_offset, const float y_offset = 0.0f,
                 const geometry_msgs::Quaternion *orientation = nullptr, const float time = 2.0f);

  /**
   * @brief moveFoot nudges foot to the given final pose in world frame
   *
   * @param side LEFT/RIGHT
   * @param foot_goal_pose a geometry_msg with desired position and orientation of the foot frame
   * @param time Time to achieve the desired goal
   * @return true
   * @return false
   */
  bool moveFoot(const RobotSide side, const geometry_msgs::Pose &foot_goal_pose, const float time);

  bool moveFoot(const RobotSide side, const std::vector<geometry_msgs::Pose> &foot_goal_poses, const float time);

  /**
   * @brief getCurrentStep gives the current position of the robot wrt to world frame
   * @param side LEFT/ RIGHT leg
   * @param foot is the foot msg which stores the location of the foot in world frame.
   */
  void getCurrentStep(int side, ihmc_msgs::FootstepDataRosMessage &foot,
                      const std::string base_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  ihmc_msgs::FootstepDataRosMessage::Ptr getOffsetStep(const RobotSide side, const geometry_msgs::Pose &goal,
                                                       const std::string base_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  /**
   * @brief raiseLeg raises the leg forward at a desired height.
   * @param side  LEFT/RIGHT
   * @param height is the height to raise the leg
   * @return
   */
  bool raiseLeg(const RobotSide side, const float height, const float time = 2.0f);

  /**
   * @brief loadEEF loads the endeffector to distribute weight evenly in both legs.
   * @param side  LEFT/RIGHT
   * @param load enum stating loading or unloading.
   * Note: After current testing, it seems that only loading condition works. Means that if the robot is in flamingo
   * position,
   * it can be bought back to normal stance position with weight evenly distributed in both legs.
   */
  void loadEEF(const RobotSide side, const EE_LOADING load);

  /**
   * @brief walkRotate rotates the robot by desired angle. this is relative to the current yaw angle.
   * @param angle is the angle expressed in radians
   * @return
   */
  bool walkRotate(const float angle, const bool waitForSteps = true);

  /**
   * @brief climbStair is a function which can make the robot climb steps given a list of step placement locations.
   * @param xOffset is a list of forward displacements.
   * @param zOffset is a list of height displacements.
   * @param startLeg LEFT/ RIGHT
   * @return
   */
  bool climbStair(const std::vector<float> &xOffset, const std::vector<float> &zOffset, const RobotSide startLeg);

  /**
   * @brief getFootstep plans footsteps to a goal.
   * @param goal is 2D goal pose
   * @param list Is a list of footstep list
   * @return
   */
  bool getFootstep(const geometry_msgs::Pose2D &goal, ihmc_msgs::FootstepDataListRosMessage &list);
  /**
   * @brief abortWalk aborts the executing footsteps
   */
  void abortWalk();

  /**
   * @brief pauseWalk pauses current footsteps by safely completing the swing or resume the footsteps that are paused.
   *
   * @param pause should be set to true to pause, or false to resume
   */
  void pauseWalk(const bool pause = true);

private:
  RobotStateInformer *current_state_;
  RobotDescription *rd_;

  const float FOOT_ALGMT_ERR_THRESHOLD = 0.03;
  const float FOOT_SEPARATION = 0.33;                     // it should be moved to robot description
  const float FOOT_ROT_ERR_THRESHOLD = 5 * M_PI / 180.0f; // 5 degrees
  double transfer_time_, swing_time_, swing_height_;
  int execution_mode_, step_counter_, step_status_;
  int previous_message_id_;
  bool isWalking_;

  ros::NodeHandle nh_;
  ros::Publisher footsteps_pub_, nudgestep_pub_, loadeff_pub, abort_footsteps_pub_, pause_walking_pub_;
  ros::Subscriber footstep_status_, walking_status_;
  ros::ServiceClient footstep_client_;
  std_msgs::String right_foot_frame_, left_foot_frame_;

  void footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage &msg);
  void walkingStatusCB(const ihmc_msgs::WalkingStatusRosMessage &msg);
  void waitForSteps();

  // /**
  //  * @brief areFeetAligned Checks if both the feet are in same orientation and in normal pose.
  //  *
  //  * @param footPose This is the footpose of one foot with respect to other
  //  * @return true if the feet are aligned within a small threshold
  //  * @return false otherwise
  //  */
  bool areFeetAligned(const geometry_msgs::Pose &footPose);
  bool isFootRotAligned(const geometry_msgs::Quaternion &q1);
  bool isFootPosAligned(const geometry_msgs::Point &p1);

  inline void initializeFootstepDataListRosMessage(ihmc_msgs::FootstepDataListRosMessage &msg, const bool queue = false)
  {
    // not supported in 0.8.2
    // if (queue && isWalking_)
    // {
    //   this->execution_mode_ = ihmc_msgs::FootstepDataListRosMessage::QUEUE;
    //   msg.previous_message_id = previous_message_id_;
    // }
    if (queue)
    {
      ROS_ERROR("Queuing of steps is not supported in 0.8.2");
    }
    msg.transfer_time = transfer_time_;
    msg.swing_time = swing_time_;
    msg.execution_mode = execution_mode_;
    previous_message_id_ = RobotWalker::id;
    msg.unique_id = RobotWalker::id++;
    if (queue)
    {
      this->execution_mode_ = ihmc_msgs::FootstepDataListRosMessage::OVERRIDE;
    }
  }

  inline void initializeFootTrajectoryRosMessage(const RobotSide side, ihmc_msgs::FootTrajectoryRosMessage &foot,
                                                 const int refFrame = TOUGH_COMMON_NAMES::WORLD_FRAME_HASH)
  {
    ihmc_msgs::SE3TrajectoryPointRosMessage data;
    // ihmc_msgs::FrameInformationRosMessage &frameInfo = foot.frame_information;

    foot.robot_side = side;
    foot.execution_mode = 0; // OVERRIDE
    foot.unique_id = id++;
    foot.taskspace_trajectory_points.push_back(data);

    // frameInfo.data_reference_frame_id = refFrame;
    // frameInfo.trajectory_reference_frame_id = refFrame;
  }

  ihmc_msgs::FootstepDataRosMessage::Ptr getOffsetStep(int side, float x, float y);
  ihmc_msgs::FootstepDataRosMessage::Ptr getOffsetStepWRTPelvis(int side, float x, float y);
};

#endif // ROBOT_WALKER_H
