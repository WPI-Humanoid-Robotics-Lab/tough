#include "tough_controller_interface/wholebody_control_interface.h"


WholebodyControlInterface::WholebodyControlInterface(ros::NodeHandle &nh):ToughControllerInterface(nh), chestController_(nh), armController_(nh)
{

    m_wholebodyPub = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>(control_topic_prefix_ + "/whole_body_trajectory", 10, true);

    rd_->getLeftArmJointLimits(joint_limits_left_);
    rd_->getRightArmJointLimits(joint_limits_right_);

    // reduce the joint limits by 1cm to avoid exceeeding limits at higher precision of float
    for (size_t i = 0; i < joint_limits_left_.size(); i++ ){
        joint_limits_left_[i] = {joint_limits_left_[i].first - 0.01, joint_limits_left_[i].second - 0.01};
        joint_limits_right_[i] = {joint_limits_right_[i].first - 0.01, joint_limits_right_[i].second - 0.01};
    }

}

void WholebodyControlInterface::executeTrajectory( const  moveit_msgs::RobotTrajectory &traj){
    return executeTrajectory( traj.joint_trajectory);
}

bool WholebodyControlInterface::getJointSpaceState(std::vector<double> &joints, RobotSide side)
{
    joints.clear();
    state_informer_->getJointPositions(joints);

    return !joints.empty();
}

bool WholebodyControlInterface::getTaskSpaceState(geometry_msgs::Pose &pose, RobotSide side, std::string fixedFrame)
{
    return state_informer_->getCurrentPose(rd_->getPelvisFrame(), pose,fixedFrame);
}

void WholebodyControlInterface::executeTrajectory(const trajectory_msgs::JointTrajectory &traj)
{

    //    if(!validateTrajectory(traj)){
    //        return;
    //    }
    ihmc_msgs::WholeBodyTrajectoryRosMessage wholeBodyMsg;
    ihmc_msgs::FrameInformationRosMessage frameInfo;

    frameInfo.data_reference_frame_id = rd_->getPelvisZUPFrameHash();
    frameInfo.trajectory_reference_frame_id = rd_->getPelvisZUPFrameHash();

    //Solving for side conflicts
    wholeBodyMsg.left_arm_trajectory_message.robot_side = LEFT;
    wholeBodyMsg.right_arm_trajectory_message.robot_side= RIGHT;

    wholeBodyMsg.left_foot_trajectory_message.robot_side = LEFT;
    wholeBodyMsg.right_foot_trajectory_message.robot_side= RIGHT;

    wholeBodyMsg.left_hand_trajectory_message.robot_side = LEFT;
    wholeBodyMsg.right_hand_trajectory_message.robot_side= RIGHT;

    // Clearing trajectory points
    wholeBodyMsg.left_arm_trajectory_message.joint_trajectory_messages.clear();
    wholeBodyMsg.right_arm_trajectory_message.joint_trajectory_messages.clear();

    // Specifying execution modes
    wholeBodyMsg.chest_trajectory_message.execution_mode   = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
    wholeBodyMsg.right_arm_trajectory_message.execution_mode = ihmc_msgs::ArmTrajectoryRosMessage::OVERRIDE;
    wholeBodyMsg.left_arm_trajectory_message.execution_mode  = ihmc_msgs::ArmTrajectoryRosMessage::OVERRIDE;

    // Setting unique id non zero for messages to be used
    wholeBodyMsg.unique_id=ros::Time::now().toSec();
    wholeBodyMsg.chest_trajectory_message.unique_id=wholeBodyMsg.unique_id;

    wholeBodyMsg.left_foot_trajectory_message.unique_id=0;//wholeBodyMsg.unique_id;
    wholeBodyMsg.right_foot_trajectory_message.unique_id=0;//wholeBodyMsg.unique_id;
    wholeBodyMsg.left_hand_trajectory_message.unique_id=0;//wholeBodyMsg.unique_id;
    wholeBodyMsg.right_hand_trajectory_message.unique_id=0;//wholeBodyMsg.unique_id;

    wholeBodyMsg.chest_trajectory_message.frame_information = frameInfo;
    wholeBodyMsg.left_foot_trajectory_message.frame_information = frameInfo;
    wholeBodyMsg.right_foot_trajectory_message.frame_information = frameInfo;
    wholeBodyMsg.pelvis_trajectory_message.frame_information = frameInfo;
    wholeBodyMsg.left_hand_trajectory_message.frame_information = frameInfo;
    wholeBodyMsg.right_hand_trajectory_message.frame_information = frameInfo;


    /*
     * While using both the chest and arm together, the chain contains 10 joints from the pelvis to palm
     * The first three joints correspond to chest yaw, chest pitch and chest roll
     * The last seven joints correspond to the joint angles in the arm
     * */

    parseTrajectory(traj);
    generateWholebodyMessage(wholeBodyMsg);
    m_wholebodyPub.publish(wholeBodyMsg);
    ROS_INFO("Published whole body msg");
    ros::Duration(0.1).sleep();
}

void WholebodyControlInterface::generateWholebodyMessage(ihmc_msgs::WholeBodyTrajectoryRosMessage &wholeBodyMsg){
    if(!chest_trajectory_.empty()) {
        chestController_.generateMessage(chest_trajectory_, 0, wholeBodyMsg.chest_trajectory_message);
    }
    if(!left_arm_trajectory_.empty()){
        armController_.generateArmMessage(RobotSide::LEFT, left_arm_trajectory_, wholeBodyMsg.left_arm_trajectory_message);
    }
    if(!right_arm_trajectory_.empty()){
        armController_.generateArmMessage(RobotSide::RIGHT, right_arm_trajectory_, wholeBodyMsg.right_arm_trajectory_message);
    }

}

void WholebodyControlInterface::generateWholebodyMessage(ihmc_msgs::WholeBodyTrajectoryRosMessage &wholeBodyMsg, const trajectory_msgs::JointTrajectory &traj){
    std::vector<std::string> left_arm_joint_names;
    std::vector<std::string> right_arm_joint_names;

    std::vector<std::pair<double, double>> left_arm_joint_limits;
    std::vector<std::pair<double, double>> right_arm_joint_limits;

    rd_->getLeftArmJointNames(left_arm_joint_names);
    rd_->getRightArmJointNames(right_arm_joint_names);

    rd_->getLeftArmJointLimits(left_arm_joint_limits);
    rd_->getRightArmJointLimits(right_arm_joint_limits);

    TrajectoryType traj_type ;
    int armIndex;
    // if 10 DOF, first 3 are chest and remaining are arm

    if (traj.joint_names.size() == 10) {
        traj_type = TrajectoryType::TEN_DOF;
        chestMsg(wholeBodyMsg.chest_trajectory_message, traj);
        armIndex = 3;

    }
    else if(traj.joint_names.size() == 7){
        traj_type = TrajectoryType::SEVEN_DOF;
        armIndex = 0;
    }
    else {
        traj_type = TrajectoryType::INVALID;
        return;
    }

    // only check the first joint and assume that planner is configured correctly!!!
    if (traj.joint_names.at(armIndex) == left_arm_joint_names.at(0)) {
        //generate left arm message
        armMsg(wholeBodyMsg.left_arm_trajectory_message,traj, left_arm_joint_limits);
    }
    else if (traj.joint_names.at(armIndex) == right_arm_joint_names.at(0)) {
        //generate right arm message
        armMsg(wholeBodyMsg.right_arm_trajectory_message, traj, right_arm_joint_limits);
    }

}

void WholebodyControlInterface::armMsg(ihmc_msgs::ArmTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<double, double> > joint_limits_)
{
    msg.joint_trajectory_messages.resize(7);
    msg.unique_id = id_++;
    for(int trajPointNumber = 0; trajPointNumber < traj.points.size(); trajPointNumber++){
        for (int jointNumber = 3; jointNumber < 10; ++jointNumber) {
            ihmc_msgs::TrajectoryPoint1DRosMessage ihmc_pointMsg;
            ihmc_pointMsg.time = traj.points[trajPointNumber].time_from_start.toSec();
            ihmc_pointMsg.position = traj.points[trajPointNumber].positions[jointNumber];
            if(ihmc_pointMsg.position <= joint_limits_[jointNumber-3].first)
            {
                ROS_WARN("Trajectory lower limit point given for %d joint",(jointNumber-3));
                ihmc_pointMsg.position=joint_limits_[jointNumber-3].first ;
            }
            else if(ihmc_pointMsg.position >= joint_limits_[jointNumber-3].second)
            {
                ROS_WARN("Trajectory upper limit point given for %d joint",(jointNumber-3));
                ihmc_pointMsg.position=joint_limits_[jointNumber-3].second ;
            }
            ihmc_pointMsg.velocity = traj.points[trajPointNumber].velocities[jointNumber];

            msg.joint_trajectory_messages[jointNumber-3].trajectory_points.push_back(ihmc_pointMsg);
            msg.joint_trajectory_messages[jointNumber-3].weight = nan("");
        }

    }
}


void WholebodyControlInterface::chestMsg(ihmc_msgs::ChestTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj)
{
    geometry_msgs::Pose pelvisPose;
    std::vector<geometry_msgs::Quaternion> quats;
    std::vector<float> timeVec;
    state_informer_->getCurrentPose(rd_->getPelvisFrame(),pelvisPose);

    for (int i = 0; i < traj.points.size(); ++i) {

        float yaw   = traj.points[i].positions[0];
        float pitch = traj.points[i].positions[1];
        float roll  = traj.points[i].positions[2];

        tf::Quaternion quatPelvis;
        quatPelvis.setRPY(roll, pitch, yaw);
        geometry_msgs::QuaternionStamped tempQuat;
        tempQuat.header.frame_id = rd_->getPelvisFrame();
        tf::quaternionTFToMsg(quatPelvis, tempQuat.quaternion);

        state_informer_->transformQuaternion(tempQuat, tempQuat);
        quats.push_back(tempQuat.quaternion);
        timeVec.push_back(traj.points.at(i).time_from_start.toSec());
    }

//    chestController_.generateMessage(quats, timeVec, 0, msg);

}

/// PENDING
bool WholebodyControlInterface::validateTrajectory(const trajectory_msgs::JointTrajectory &traj)
{
    for (trajectory_msgs::JointTrajectoryPoint point : traj.points){
        if (point.positions.size() != 10 && point.positions.size() != 11){
            ROS_INFO("Points size mismatched in wholebody controller");
            return false;
        }
    }
    return true;
}



void WholebodyControlInterface::parseTrajectory(const trajectory_msgs::JointTrajectory &traj)
{

    // reset all trajectory points
    chest_trajectory_.resize(0);
    left_arm_trajectory_.resize(0);
    right_arm_trajectory_.resize(0);

    // get all related parameters and limits
    std::vector<std::string> left_arm_joint_names;
    std::vector<std::string> right_arm_joint_names;
    std::vector<std::string> chest_joint_names;

    std::vector<std::pair<double, double>> left_arm_joint_limits;
    std::vector<std::pair<double, double>> right_arm_joint_limits;

    rd_->getLeftArmJointNames(left_arm_joint_names);
    rd_->getRightArmJointNames(right_arm_joint_names);
    rd_->getChestJointNames(chest_joint_names);

    rd_->getLeftArmJointLimits(left_arm_joint_limits);
    rd_->getRightArmJointLimits(right_arm_joint_limits);

    long chest_start=-1, l_arm_start=-1, r_arm_start=-1;

    auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), chest_joint_names.at(0));
    if(it == traj.joint_names.end()){
        // does not contain chest points
    }
    else {
        // set the chest indices
        chest_start = std::distance(traj.joint_names.begin(), it);
    }

    it = std::find(traj.joint_names.begin(), traj.joint_names.end(), left_arm_joint_names.at(0));
    if(it == traj.joint_names.end()){
        // does not contain left arm points
    }
    else {
        // set the left arm indices
        l_arm_start = std::distance(traj.joint_names.begin(), it);
        left_arm_trajectory_.resize(left_arm_joint_names.size());
    }

    it = std::find(traj.joint_names.begin(), traj.joint_names.end(), right_arm_joint_names.at(0));
    if(it == traj.joint_names.end()){
        // does not contain left arm points
    }
    else {
        // set the right arm indices
        r_arm_start = std::distance(traj.joint_names.begin(), it);
        right_arm_trajectory_.resize(right_arm_joint_names.size());
    }

    double traj_point_time = 0.0;
    for (size_t i = 0; i < traj.points.size(); i++) {
        traj_point_time =traj.points.at(i).time_from_start.toSec();

        //chest - move to a new function later
        if(chest_start >= 0) {
            ihmc_msgs::SO3TrajectoryPointRosMessage chest_orientation;
            createChestQuaternion(chest_start, traj.points.at(i), chest_orientation.orientation);
            chest_orientation.time = traj_point_time;
            chest_trajectory_.push_back(chest_orientation);
        }
        //arm
        if(l_arm_start >= 0) {
            appendArmPoint(l_arm_start, traj.points.at(i), left_arm_joint_limits, left_arm_trajectory_);
        }

        if(r_arm_start >= 0) {
            appendArmPoint(r_arm_start, traj.points.at(i), right_arm_joint_limits, right_arm_trajectory_);
        }

    }
}



/*
 * Validate joints in the trajectory
 * auto it_temp = it;

        for (auto chest_joint_it = chest_joint_names.begin(); chest_joint_it != chest_joint_names.end(); ++chest_joint_it, ++it_temp)
        {
            if(*it_temp != *chest_joint_it) {
                // joints in trajectory are not in the expected sequence.
            }
        }
 */
