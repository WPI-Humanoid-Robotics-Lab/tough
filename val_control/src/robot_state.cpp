#include "val_control/robot_state.h"

RobotStateInformer* RobotStateInformer::currentObject_ = nullptr;

/* Singleton implementation */
RobotStateInformer* RobotStateInformer::getRobotStateInformer(ros::NodeHandle nh){

    // check if an object of this class already exists, if not create one
    if(RobotStateInformer::currentObject_ == nullptr){
        RobotStateInformer::currentObject_ = new RobotStateInformer(nh);
    }
    return RobotStateInformer::currentObject_;
}

RobotStateInformer::RobotStateInformer(ros::NodeHandle nh):nh_(nh){
    ROS_INFO("Object Created");
    jointStateSub_ = nh_.subscribe("/joint_states", 1, &RobotStateInformer::jointStateCB, this);
    ros::Duration(0.2).sleep();
}

RobotStateInformer::~RobotStateInformer(){
    delete currentObject_;
}

void RobotStateInformer::jointStateCB(const sensor_msgs::JointStatePtr msg){

    std::lock_guard<std::mutex> guard(currentStateMutex_);
    for(size_t i = 0; i < msg->name.size(); ++i){
        RobotState state;
        state.name     = msg->name[i];
        state.position = msg->position[i];
        state.velocity = msg->velocity[i];
        state.effort   = msg->effort[i];
        currentState_[msg->name[i]] = state;
    }

}

void RobotStateInformer::getJointPositions(std::vector<float> &positions){
    positions.clear();
    std::lock_guard<std::mutex> guard(currentStateMutex_);

    for (auto it = currentState_.begin(); it != currentState_.end(); ++it){
        positions.push_back(it->second.position);
    }
}

bool RobotStateInformer::getJointPositions(const std::string &paramName, std::vector<float> &positions){
    positions.clear();
    std::vector<std::string> jointNames;
    std::string parameter;
    if(paramName == "left_arm_joint_names" || paramName == "left_arm"){
        parameter.assign("/ihmc_ros/valkyrie/left_arm_joint_names");
    }
    else if (paramName == "right_arm_joint_names"  || paramName == "right_arm"){
        parameter.assign("/ihmc_ros/valkyrie/right_arm_joint_names");
    }
    else{
        parameter.assign(paramName);
    }

    std::lock_guard<std::mutex> guard(currentStateMutex_);
    if (nh_.getParam(parameter, jointNames)){
        for (auto joint : jointNames){
            positions.push_back((currentState_[joint]).position);
        }
        return true;
    }
    return false;
}

void RobotStateInformer::getJointVelocities(std::vector<float> &velocities){
    velocities.clear();
    std::lock_guard<std::mutex> guard(currentStateMutex_);

    for (auto it = currentState_.begin(); it != currentState_.end(); ++it){
        velocities.push_back(it->second.velocity);
    }
}

bool RobotStateInformer::getJointVelocities(const std::string &paramName, std::vector<float> &velocities){
    velocities.clear();
    std::vector<std::string> jointNames;
    std::string parameter;
    if(paramName == "left_arm_joint_names" || paramName == "left_arm"){
        parameter.assign("/ihmc_ros/valkyrie/left_arm_joint_names");
    }
    else if (paramName == "right_arm_joint_names"  || paramName == "right_arm"){
        parameter.assign("/ihmc_ros/valkyrie/right_arm_joint_names");
    }
    else{
        parameter.assign(paramName);
    }

    std::lock_guard<std::mutex> guard(currentStateMutex_);
    if (nh_.getParam(parameter, jointNames)){
        for (auto joint : jointNames){
            velocities.push_back((currentState_[joint]).velocity);
        }
        return true;
    }
    return false;
}


void RobotStateInformer::getJointEfforts(std::vector<float> &efforts){
    efforts.clear();
    std::lock_guard<std::mutex> guard(currentStateMutex_);

    for (auto it = currentState_.begin(); it != currentState_.end(); ++it){
        efforts.push_back(it->second.effort);
    }
}

bool RobotStateInformer::getJointEfforts(const std::string &paramName, std::vector<float> &efforts){
    efforts.clear();
    std::vector<std::string> jointNames;
    std::string parameter;
    if(paramName == "left_arm_joint_names" || paramName == "left_arm"){
        parameter.assign("/ihmc_ros/valkyrie/left_arm_joint_names");
    }
    else if (paramName == "right_arm_joint_names"  || paramName == "right_arm"){
        parameter.assign("/ihmc_ros/valkyrie/right_arm_joint_names");
    }
    else{
        parameter.assign(paramName);
    }

    std::lock_guard<std::mutex> guard(currentStateMutex_);
    if (nh_.getParam(parameter, jointNames)){
        for (auto joint : jointNames){
            efforts.push_back((currentState_[joint]).effort);
        }
        return true;
    }
    return false;
}


double RobotStateInformer::getJointPosition(const std::string &jointName)
{
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    return (currentState_[jointName]).position;
}

double RobotStateInformer::getJointVelocity(const std::string &jointName)
{
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    return (currentState_[jointName]).velocity;
}

double RobotStateInformer::getJointEffort(const std::string &jointName)
{
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    return (currentState_[jointName]).effort;
}

void RobotStateInformer::getJointNames(std::vector<std::string> &jointNames)
{
    jointNames.clear();
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    for (auto i : currentState_){
        jointNames.push_back(i.first);
    }
}

bool RobotStateInformer::getCurrentPose(const std::string &frameName, geometry_msgs::Pose &pose, const std::string &baseFrame)
{
    tf::StampedTransform origin;

    try {

        listener_.waitForTransform(baseFrame, frameName, ros::Time(0), ros::Duration(2));
        listener_.lookupTransform(baseFrame, frameName, ros::Time(0),origin);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    tf::pointTFToMsg(origin.getOrigin(), pose.position);
    tf::quaternionTFToMsg(origin.getRotation(), pose.orientation);

    return true;
}


bool RobotStateInformer::transformQuaternion(const geometry_msgs::QuaternionStamped &qt_in, geometry_msgs::QuaternionStamped &qt_out,const std::string target_frame)
{
    try{

        listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformQuaternion(target_frame, qt_in, qt_out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    return true;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::PointStamped &pt_in, geometry_msgs::PointStamped &pt_out,const std::string target_frame)
{
    try{

        listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformPoint(target_frame, pt_in, pt_out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    return true;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::Point &pt_in, geometry_msgs::Point &pt_out,const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::PointStamped stmp_pt_in, stmp_pt_out;
    stmp_pt_in.header.frame_id = from_frame;
    stmp_pt_in.point = pt_in;

    try{

        listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformPoint(to_frame, stmp_pt_in, stmp_pt_out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    pt_out = stmp_pt_out.point;
    return true;
}


bool RobotStateInformer::isHandleInGrasp(armSide side)
{
    // fill this in :)
    return true;
}
