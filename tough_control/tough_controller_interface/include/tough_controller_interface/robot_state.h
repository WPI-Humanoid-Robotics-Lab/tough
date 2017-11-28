#ifndef VAL_ROBOT_STATE_H
#define VAL_ROBOT_STATE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <tf/transform_listener.h>
#include <mutex>
#include "tough_common/val_common_names.h"
#include "tough_common/val_common_defines.h"
#include <geometry_msgs/Pose2D.h>
#include "tough_common/robot_description.h"

struct RobotState{
    std::string name;
    float position;
    float velocity;
    float effort;
};

class RobotStateInformer{
private:
    // private constructor to disable user from creating objects
    RobotStateInformer(ros::NodeHandle nh);
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    static RobotStateInformer* currentObject_;
    RobotDescription *rd_;

    ros::Subscriber jointStateSub_;
    void jointStateCB(const sensor_msgs::JointStatePtr msg);

     std::map<std::string, RobotState> currentState_;
     std::mutex currentStateMutex_;
     std::string robotName_;
    ~RobotStateInformer();

public:
    static RobotStateInformer* getRobotStateInformer(ros::NodeHandle nh);

    void getJointPositions(std::vector<float> &positions);
    bool getJointPositions(const std::string &paramName, std::vector<float> &positions);

    void getJointVelocities(std::vector<float> &velocities);
    bool getJointVelocities(const std::string &paramName, std::vector<float> &velocities);

    void getJointEfforts(std::vector<float> &efforts);
    bool getJointEfforts(const std::string &paramName, std::vector<float> &efforts);

    double getJointPosition(const std::string &jointName);
    double getJointVelocity(const std::string &jointName);
    double getJointEffort(const std::string &jointName);

    void getJointNames(std::vector<std::string> &jointNames);

    bool getCurrentPose(const std::string &frameName, geometry_msgs::Pose &pose, const std::string &baseFrame=VAL_COMMON_NAMES::WORLD_TF);

    bool transformQuaternion(const geometry_msgs::QuaternionStamped &qt_in, geometry_msgs::QuaternionStamped &qt_out,const std::string target_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformQuaternion(const geometry_msgs::Quaternion &qt_in, geometry_msgs::Quaternion &qt_out,const std::string &from_frame, const std::string &to_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformPoint(const geometry_msgs::PointStamped &pt_in, geometry_msgs::PointStamped &pt_out,const std::string target_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformPoint(const geometry_msgs::Point &pt_in, geometry_msgs::Point &pt_out,const std::string &from_frame, const std::string &to_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformPose(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out,const std::string &from_frame, const std::string &to_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformPose(const geometry_msgs::Pose2D &pose_in, geometry_msgs::Pose2D &pose_out,const std::string &from_frame, const std::string &to_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformVector(const geometry_msgs::Vector3 &vec_in, geometry_msgs::Vector3 &vec_out,const std::string &from_frame, const std::string &to_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool transformVector(const geometry_msgs::Vector3Stamped &vec_in, geometry_msgs::Vector3Stamped &vec_out,const std::string target_frame=VAL_COMMON_NAMES::WORLD_TF);
    bool isGraspped(RobotSide side);
    std::vector<float> closeRightGrasp,closeLeftGrasp,openGrasp;


};

#endif // VAL_ROBOT_STATE_H
