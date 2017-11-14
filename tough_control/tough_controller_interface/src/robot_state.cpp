#include "tough_controller_interface/robot_state.h"

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
//    ROS_INFO("Object Created");
    jointStateSub_ = nh_.subscribe("/joint_states", 1, &RobotStateInformer::jointStateCB, this);
    ros::Duration(0.2).sleep();
    closeRightGrasp={1.09,1.47,1.84,0.90,1.20,1.51,0.99,1.34,1.68,0.55,0.739,0.92,1.40};
    closeLeftGrasp={0.0,-1.47,-1.84,-0.90,-1.20,-1.51,-0.99,-1.34,-1.68,-0.55,-0.739,-0.92,1.40};
    openGrasp={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    rd_ = RobotDescription::getRobotDescription(nh_);
    nh.getParam("ihmc_ros/robot_name", robotName_);

}

RobotStateInformer::~RobotStateInformer(){
    jointStateSub_.shutdown();
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
        parameter.assign("/ihmc_ros/"+robotName_ + "/left_arm_joint_names");
    }
    else if (paramName == "right_arm_joint_names"  || paramName == "right_arm"){
        parameter.assign("/ihmc_ros/"+robotName_ + "/right_arm_joint_names");
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
        parameter.assign("/ihmc_ros/"+robotName_ + "/left_arm_joint_names");
    }
    else if (paramName == "right_arm_joint_names"  || paramName == "right_arm"){
        parameter.assign("/ihmc_ros/"+robotName_ + "/right_arm_joint_names");
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
        parameter.assign("/ihmc_ros/"+robotName_ + "/left_arm_joint_names");
    }
    else if (paramName == "right_arm_joint_names"  || paramName == "right_arm"){
        parameter.assign("/ihmc_ros/"+robotName_ + "/right_arm_joint_names");
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

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformQuaternion(target_frame, qt_in, qt_out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    return true;
}

bool RobotStateInformer::transformQuaternion(const geometry_msgs::Quaternion &qt_in, geometry_msgs::Quaternion &qt_out,const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::QuaternionStamped in, out;
    in.quaternion = qt_in;
    in.header.frame_id = from_frame;
    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformQuaternion(to_frame, in, out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    qt_out = out.quaternion;
    return true;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::PointStamped &pt_in, geometry_msgs::PointStamped &pt_out,const std::string target_frame)
{
    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformPoint(target_frame, pt_in, pt_out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    return true;
}

bool RobotStateInformer::transformPose(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out,const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::PoseStamped in, out;
    in.header.frame_id = from_frame;
    in.header.stamp = ros::Time(0);
    in.pose = pose_in;
    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformPose(to_frame, in, out);
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    pose_out = out.pose;
    return true;
}

bool RobotStateInformer::transformPose(const geometry_msgs::Pose2D &pose_in, geometry_msgs::Pose2D &pose_out,const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::PoseStamped in, out;
    in.header.frame_id = from_frame;
    in.header.stamp = ros::Time(0);
    in.pose.position.x = pose_in.x;
    in.pose.position.y = pose_in.y;
    in.pose.position.z = 0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose_in.theta), in.pose.orientation);

    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformPose(to_frame, in, out);
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    pose_out.x     = out.pose.position.x;
    pose_out.y     = out.pose.position.y;
    pose_out.theta = tf::getYaw(out.pose.orientation);

    return true;
}

bool RobotStateInformer::transformPoint(const geometry_msgs::Point &pt_in, geometry_msgs::Point &pt_out,const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::PointStamped stmp_pt_in, stmp_pt_out;
    stmp_pt_in.header.frame_id = from_frame;
    stmp_pt_in.point = pt_in;

    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
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

bool RobotStateInformer::transformVector(const geometry_msgs::Vector3Stamped &vec_in, geometry_msgs::Vector3Stamped &vec_out,const std::string target_frame)
{
    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformVector(target_frame, vec_in, vec_out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    return true;
}


bool RobotStateInformer::transformVector(const geometry_msgs::Vector3 &vec_in, geometry_msgs::Vector3 &vec_out,const std::string &from_frame, const std::string &to_frame)
{
    geometry_msgs::Vector3Stamped in, out;
    in.vector = vec_in;
    in.header.frame_id = from_frame;
    try{

        listener_.waitForTransform(rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        listener_.transformVector(to_frame, in, out);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }
    vec_out = out.vector;
    return true;
}

bool RobotStateInformer::isGraspped(RobotSide side)
{
    std::vector<float> jointPos,closeGrasp;
    closeGrasp= side == LEFT ? closeLeftGrasp : closeRightGrasp;

    if(side ==RIGHT)
    {
        jointPos.push_back(getJointPosition("rightIndexFingerPitch1"));
        jointPos.push_back(getJointPosition("rightIndexFingerPitch2"));
        jointPos.push_back(getJointPosition("rightIndexFingerPitch3"));
        jointPos.push_back(getJointPosition("rightMiddleFingerPitch1"));
        jointPos.push_back(getJointPosition("rightMiddleFingerPitch2"));
        jointPos.push_back(getJointPosition("rightMiddleFingerPitch3"));
        jointPos.push_back(getJointPosition("rightPinkyPitch1"));
        jointPos.push_back(getJointPosition("rightPinkyPitch2"));
        jointPos.push_back(getJointPosition("rightPinkyPitch3"));
        jointPos.push_back(getJointPosition("rightThumbPitch1"));
        jointPos.push_back(getJointPosition("rightThumbPitch2"));
        jointPos.push_back(getJointPosition("rightThumbPitch3"));
        jointPos.push_back(getJointPosition("rightThumbRoll"));
    }
    else
    {
        jointPos.push_back(getJointPosition("leftIndexFingerPitch1"));
        jointPos.push_back(getJointPosition("leftIndexFingerPitch2"));
        jointPos.push_back(getJointPosition("leftIndexFingerPitch3"));
        jointPos.push_back(getJointPosition("leftMiddleFingerPitch1"));
        jointPos.push_back(getJointPosition("leftMiddleFingerPitch2"));
        jointPos.push_back(getJointPosition("leftMiddleFingerPitch3"));
        jointPos.push_back(getJointPosition("leftPinkyPitch1"));
        jointPos.push_back(getJointPosition("leftPinkyPitch2"));
        jointPos.push_back(getJointPosition("leftPinkyPitch3"));
        jointPos.push_back(getJointPosition("leftThumbPitch1"));
        jointPos.push_back(getJointPosition("leftThumbPitch2"));
        jointPos.push_back(getJointPosition("leftThumbPitch3"));
        jointPos.push_back(getJointPosition("leftThumbRoll"));
    }

    float diffClose=0.0,diffOpen=0.0;

    for (size_t i = 0; i < closeGrasp.size(); ++i)
    {
        diffClose+=fabs(jointPos[i]-closeGrasp[i]);
        diffOpen+=fabs(jointPos[i]-openGrasp[i]);
    }

    if(fabs(diffOpen)<0.1)
    {
        return false;
    }
    else if(fabs(diffClose)<0.1)
    {
        return false;
    }
    else return true;

}
