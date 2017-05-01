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
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    if (nh_.getParam(paramName, jointNames)){
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
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    if (nh_.getParam(paramName, jointNames)){
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
    std::lock_guard<std::mutex> guard(currentStateMutex_);
    if (nh_.getParam(paramName, jointNames)){
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
