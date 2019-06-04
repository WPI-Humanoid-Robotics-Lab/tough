#include <ros/ros.h>
#include <ihmc_msgs/FootTrajectoryRosMessage.h>
#include <ihmc_msgs/FootstepDataRosMessage.h>
#include <ihmc_msgs/FootLoadBearingRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <tough_common/robot_state.h>
#include "tough_common/robot_description.h"
#include "tough_controller_interface/tough_control_interface.h"

class LegControlInterface : public ToughControlInterface
{
public:
  LegControlInterface(ros::NodeHandle nh);
  ~LegControlInterface();
  virtual bool getJointSpaceState(std::vector<double>& joints, RobotSide side);
  virtual bool getTaskSpaceState(geometry_msgs::Pose& pose, RobotSide side, std::string fixedFrame);
  void moveFoot(const RobotSide side, const std::vector<geometry_msgs::Pose>& foot_goal_poses, const float time);
  void moveFoot(const RobotSide side, const geometry_msgs::Pose& foot_goal_pose, const float time);
  void raiseLeg(const RobotSide side, const float offset, const float time);
  void placeLeg(const RobotSide side, const float offset, const float time);
  void curlLeg(RobotSide side, float radius, float time);

private:
  ros::Publisher legTrajectoryPub_, loadEndEffPub_;

  inline void initializeFootTrajectoryRosMessage(const RobotSide side, ihmc_msgs::FootTrajectoryRosMessage& foot,
                                                 const int refFrame = TOUGH_COMMON_NAMES::WORLD_FRAME_HASH)
  {
    ihmc_msgs::SE3TrajectoryPointRosMessage data;
    ihmc_msgs::FrameInformationRosMessage& frameInfo = foot.frame_information;

    foot.robot_side = side;
    foot.execution_mode = 0;  // OVERRIDE
    foot.unique_id = id_++;
    foot.taskspace_trajectory_points.push_back(data);

    frameInfo.data_reference_frame_id = refFrame;
    frameInfo.trajectory_reference_frame_id = refFrame;
  }
};