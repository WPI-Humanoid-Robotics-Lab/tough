#ifndef TOUGH_ROBOT_STATE_INFORMER_H
#define TOUGH_ROBOT_STATE_INFORMER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <tf/transform_listener.h>
#include <mutex>
#include <geometry_msgs/Pose2D.h>
#include "tough_common/robot_description.h"
#include <sensor_msgs/Imu.h>
#include <ihmc_msgs/Point2dRosMessage.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

struct RobotState
{
  std::string name;
  float position;
  float velocity;
  float effort;
};

class RobotStateInformer
{
private:
  // private constructor to disable user from creating objects
  RobotStateInformer(ros::NodeHandle nh);
  static RobotStateInformer* currentObject_;
  RobotDescription* rd_;

  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string robotName_;

  ros::Subscriber jointStateSub_;
  void jointStateCB(const sensor_msgs::JointState::Ptr msg);
  sensor_msgs::JointState::Ptr currentStatePtr_;
  std::map<std::string, RobotState> currentState_;
  std::vector<std::string> jointNames_;
  std::mutex currentStateMutex_;

  ros::Subscriber pelvisIMUSub_;
  void pelvisImuCB(const sensor_msgs::Imu::Ptr msg);
  sensor_msgs::Imu::Ptr pelvisImuValue_;

  ros::Subscriber centerOfMassSub_;
  void centerOfMassCB(const geometry_msgs::Point32::Ptr msg);
  geometry_msgs::Point32::Ptr centerOfMassValue_;

  ros::Subscriber capturePointSub_;
  void capturPointCB(const ihmc_msgs::Point2dRosMessage::Ptr msg);
  ihmc_msgs::Point2dRosMessage::Ptr capturePointValue_;

  ros::Subscriber isInDoubleSupportSub_;
  void doubleSupportStatusCB(const std_msgs::Bool& msg);
  bool doubleSupportStatus_;

  ros::Subscriber leftFootForceSensorSub_;
  ros::Subscriber rightFootForceSensorSub_;
  void leftFootForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg);
  void rightFootForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg);
  std::map<RobotSide, geometry_msgs::WrenchStamped::Ptr> footWrenches_;

  ros::Subscriber leftWristForceSensorSub_;
  ros::Subscriber rightWristForceSensorSub_;
  void leftWristForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg);
  void rightWristForceSensorCB(const geometry_msgs::WrenchStamped::Ptr msg);
  std::map<RobotSide, geometry_msgs::WrenchStamped::Ptr> wristWrenches_;

  void populateStateMap();
  void initializeClassMembers();

  void inline parseParameter(const std::string& paramName, std::string& parameter)
  {
    if (paramName == "left_arm_joint_names" || paramName == "left_arm")
    {
      parameter.assign(TOUGH_COMMON_NAMES::TOPIC_PREFIX + robotName_ + "/" +
                       TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM);
    }
    else if (paramName == "right_arm_joint_names" || paramName == "right_arm")
    {
      parameter.assign(TOUGH_COMMON_NAMES::TOPIC_PREFIX + robotName_ + "/" +
                       TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM);
    }
    else
    {
      parameter.assign(paramName);
    }
  }

public:
  /**
   * @brief Get the Robot State Informer object. Use this function to access a pointer to the object of
   * RobotStateInformer. Only one object of this class is created and it is shared with all the classes that use
   * RobotStateInformer.
   *
   * @param nh ros Nodehandle
   * @return RobotStateInformer*
   */
  static RobotStateInformer* getRobotStateInformer(ros::NodeHandle nh);
  ~RobotStateInformer();

  /**
   * @brief disable copy constructor. This is required for singleton pattern
   *
   */
  RobotStateInformer(RobotStateInformer const&) = delete;
  /**
   * @brief disable assignment operator. This is required for singleton pattern
   *
   */
  void operator=(RobotStateInformer const&) = delete;

  /**
   * @brief Get the current joint state message.
   *
   * @param jointState        [output]
   */
  void getJointStateMessage(sensor_msgs::JointState& jointState);

  /**
   * @brief The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their 
   * vectors are same. So any perticular joint will have a same index in all of the vectors. 
   * 
   * This Method returns the Joint Number of a Joint Name supplied. 
   *
   * @param jointName
   * @return int
   */
  int getJointNumber(std::string jointName);

  /**
   * @brief Get the current positions of all joints. Ordering is based on the order in the JointNames vector.
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same. 
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param positions         [output]
   */
  void getJointPositions(std::vector<double>& positions);
  /**
   * @brief Get the current positions of joint names present in the parameter
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param paramName         - Parameter on ros param server that has an array of joint names
   * @param positions         [output]
   * @return true             when successful
   * @return false
   */
  bool getJointPositions(const std::string& paramName, std::vector<double>& positions);

  /**
   * @brief Get the current velocities of joints
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param velocities [output]
   */
  void getJointVelocities(std::vector<double>& velocities);

  /**
   * @brief Get the Joint Velocities of joint names present in the parameter
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same. 
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param paramName       - Parameter name on ros param server that has an array of joint names
   * @param velocities      [output]
   * @return true           - when successful
   * @return false 
   */
  bool getJointVelocities(const std::string& paramName, std::vector<double>& velocities);

  /**
   * @brief Get the vector of Joint Efforts for all the joints.
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param efforts         [output]
   */
  void getJointEfforts(std::vector<double>& efforts);

  /**
   * @brief Get the vector of Joint Efforts for all the joints.
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param paramName
   * @param efforts
   * @return true
   * @return false
   */
  bool getJointEfforts(const std::string& paramName, std::vector<double>& efforts);

  /**
   * @brief Get the Joint Position for the joint of jointName
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointName
   * @return double
   */
  double getJointPosition(const std::string& jointName);

  /**
   * @brief Get the Joint Position for the joint at jointNumber
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointNumber
   * @return double
   */
  double getJointPosition(const int jointNumber);

  /**
   * @brief Get the Joint Velocity for the joint of jointName
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointName
   * @return double
   */
  double getJointVelocity(const std::string& jointName);

  /**
   * @brief Get the Joint Velocity for the joint at jointNumber
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointNumber
   * @return double
   */
  double getJointVelocity(const int jointNumber);

  /**
   * @brief Get the Joint Effort for the joint of jointName
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointName
   * @return double
   */
  double getJointEffort(const std::string& jointName);

  /**
   * @brief Get the Joint Effort for the joint at jointNumber
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointNumber
   * @return double
   */
  double getJointEffort(const int jointNumber);

  /**
   * @brief Get the vector of Joint Names for all the joints
   * The order for the Joints' Names, Numbers, Positions, Velocities and Efforts in their vectors are same.
   * So any perticular joint will have a same index in all of the vectors.
   *
   * @param jointNames
   */
  void getJointNames(std::vector<std::string>& jointNames);

  /**
   * @brief Get the Current Pose of the frameName with respect to the baseFrame
   * 
   * @param frameName               - The name of the required frame, whose pose is to be found
   * @param pose                    - Pose of the frameName wrt baseFrame [output]
   * @param baseFrame               - The name of the Reference frame
   * @return true                   - when successful
   * @return false 
   */
  bool getCurrentPose(const std::string& frameName, geometry_msgs::Pose& pose,
                      const std::string& baseFrame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Get the Transform from baseFrame to the frameName.
   *
   * @param frameName               - The name of the required frame, whose transform is to be found
   * @param transform               - Final transform from baseFrame to the frameName [output]
   * @param baseFrame               - The name of the reference frame
   * @return true                   - When successful
   * @return false
   */
  bool getTransform(const std::string& frameName, tf::StampedTransform& transform,
                    const std::string& baseFrame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the quaternion from the current reference frame to the target_frame
   * 
   * @param qt_in                   - Input Quaternion for the transformation
   * @param qt_out                  - Output Quaternion after the transformation [output]
   * @param target_frame            - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false 
   */
  bool transformQuaternion(const geometry_msgs::QuaternionStamped& qt_in, geometry_msgs::QuaternionStamped& qt_out,
                           const std::string target_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  
  /**
   * @brief Transforms the quaternion from the from_frame to the to_frame
   *
   * @param qt_in                   - Input Quaternion for the transformation
   * @param qt_out                  - Output Quaternion after the transformation [output]
   * @param from_frame              - Current Reference frame before the transformation.
   * @param to_frame                - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformQuaternion(const geometry_msgs::Quaternion& qt_in, geometry_msgs::Quaternion& qt_out,
                           const std::string& from_frame, const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the point from the current reference frame to the target_frame
   *
   * @param pt_in                   - Input Point for the transformation
   * @param pt_out                  - Output Point after the transformation [output]
   * @param target_frame            - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformPoint(const geometry_msgs::PointStamped& pt_in, geometry_msgs::PointStamped& pt_out,
                      const std::string target_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the point from the from_frame frame to the to_frame
   *
   * @param pt_in                   - Input Point for the transformation
   * @param pt_out                  - Output Point after the transformation [output]
   * @param from_frame              - Current Reference frame Before thr transformation
   * @param to_frame                - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformPoint(const geometry_msgs::Point& pt_in, geometry_msgs::Point& pt_out, const std::string& from_frame,
                      const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the pose from the current reference frame to the target_frame
   *
   * @param pose_in                 - Input Pose for the transformation
   * @param pose_out                - Output Pose after the transformation [output]
   * @param to_frame                - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformPose(const geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped& pose_out,
                     const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the pose from the from_frame frame to the to_frame
   *
   * @param pose_in                 - Input Pose for the transformation
   * @param pose_out                - Output Pose after the transformation [output]
   * @param from_frame              - Current Reference frame befor the transformation.
   * @param to_frame                - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out, const std::string& from_frame,
                     const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the pose from the from_frame frame to the to_frame
   *
   * @param pose_in                 - Input Pose for the transformation
   * @param pose_out                - Output Pose after the transformation [output]
   * @param from_frame              - Current Reference frame befor the transformation.
   * @param to_frame                - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformPose(const geometry_msgs::Pose2D& pose_in, geometry_msgs::Pose2D& pose_out,
                     const std::string& from_frame, const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Transforms the vector3 message from the from_frame frame to the to_frame
   *
   * @param vec_in                  - Input Vector for the transformation
   * @param vec_out                 - Output Vector after the transformation [output]
   * @param from_frame              - Current Reference frame befor the transformation.
   * @param to_frame                - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformVector(const geometry_msgs::Vector3& vec_in, geometry_msgs::Vector3& vec_out,
                       const std::string& from_frame, const std::string& to_frame = TOUGH_COMMON_NAMES::WORLD_TF);
  
  /**
   * @brief Transforms the vector3 message from the current frame to the target_frame
   *
   * @param vec_in                  - Input Vector for the transformation
   * @param vec_out                 - Output Vector after the transformation [output]
   * @param target_frame            - Reference frame for the transformation
   * @return true                   - When Successful
   * @return false
   */
  bool transformVector(const geometry_msgs::Vector3Stamped& vec_in, geometry_msgs::Vector3Stamped& vec_out,
                       const std::string target_frame = TOUGH_COMMON_NAMES::WORLD_TF);

  /**
   * @brief Get the Wrenches on the both the feet.
   * The result is mapped with with the RobotSide, RIGHT and LEFT.
   *
   * @param wrenches                - [output]
   */
  void getFootWrenches(std::map<RobotSide, geometry_msgs::Wrench>& wrenches);
  
  /**
   * @brief Get the Wrenches on the both the wrists.
   * The result is mapped with with the RobotSide, RIGHT and LEFT.
   *
   * @param wrenches                - [output]
   */
  void getWristWrenches(std::map<RobotSide, geometry_msgs::Wrench>& wrenches);

  /**
   * @brief Get the Wrenches on the foot
   *
   * @param side                    - Side of the robot. It can be RIGHT or LEFT.
   * @param wrench                  - [output]
   */
  void getFootWrench(const RobotSide side, geometry_msgs::Wrench& wrench);

  /**
   * @brief Get the Wrenches on the wrists
   *
   * @param side                    - Side of the robot. It can be RIGHT or LEFT.
   * @param wrench                  - [output]
   */
  void getWristWrench(const RobotSide side, geometry_msgs::Wrench& wrench);

  /**
   * @brief Get the Forces on the foot
   *
   * @param side                    - Side of the robot. It can be RIGHT or LEFT.
   * @param force                   - [output]
   */
  void getFootForce(const RobotSide side, geometry_msgs::Vector3& force);

  /**
   * @brief Get the Torques on the foot
   *
   * @param side                    - Side of the robot. It can be RIGHT or LEFT.
   * @param torque                  - [output]
   */
  void getFootTorque(const RobotSide side, geometry_msgs::Vector3& torque);

  /**
   * @brief Get the Forces on the Wrist
   *
   * @param side                    - Side of the robot. It can be RIGHT or LEFT.
   * @param force                   - [output]
   */
  void getWristForce(const RobotSide side, geometry_msgs::Vector3& force);

  /**
   * @brief Get the Torques on the Wrist
   *
   * @param side                    - Side of the robot. It can be RIGHT or LEFT.
   * @param torque                  - [output]
   */
  void getWristTorque(const RobotSide side, geometry_msgs::Vector3& torque);

  /**
   * @brief     If the robot has both of its feet in contact with the ground, the robot is 
   * said to be in double support. 
   * 
   * @return true                   - If the robot is in double support
   * @return false                  - If the robot is not in the double support
   */
  bool isRobotInDoubleSupport();

  /**
   * @brief Get the position of Capture Point of the robot. A Capture Point is a point on the ground in which the 
   * Center of Pressure can be placed in order to stop a robot
   *
   * @param point                   - [output]
   */
  void getCapturePoint(geometry_msgs::Point& point);

  /**
   * @brief Get the position of Center Of Mass of the robot.
   * 
   * @param point                   - [output]
   */
  void getCenterOfMass(geometry_msgs::Point& point);

  /**
   * @brief Get the Pelvis IMU Reading 
   * 
   * @param msg                     - [output]
   */
  void getPelvisIMUReading(sensor_msgs::Imu& msg);
};

#endif  // TOUGH_ROBOT_STATE_INFORMER_H
