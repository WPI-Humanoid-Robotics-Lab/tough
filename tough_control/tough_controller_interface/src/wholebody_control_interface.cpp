#include "tough_controller_interface/wholebody_control_interface.h"


WholebodyControlInterface::WholebodyControlInterface(ros::NodeHandle &nh):ToughControllerInterface(nh)
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

void WholebodyControlInterface::executeTrajectory(const RobotSide side, const  moveit_msgs::RobotTrajectory &traj){
    return executeTrajectory(side, traj.joint_trajectory);
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

void WholebodyControlInterface::executeTrajectory(const RobotSide side, const trajectory_msgs::JointTrajectory &traj)
{

    if(!validateTrajectory(traj)){
        return;
    }
    ihmc_msgs::WholeBodyTrajectoryRosMessage wholeBodyMsg;

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

    /*
     * While using both the chest and arm together, the chain contains 10 joints from the pelvis to palm
     * The first three joints correspond to chest yaw, chest pitch and chest roll
     * The last seven joints correspond to the joint angles in the arm
     * */

    ArmControlInterface arm(nh_);


    // Arm
    if(side == LEFT)
    {
        wholeBodyMsg.left_arm_trajectory_message.unique_id=wholeBodyMsg.unique_id;
        leftArmMsg(wholeBodyMsg,traj,joint_limits_left_);
        //        arm.generateArmMessage(side, poses, traj.points.at(0).time_from_start , wholeBodyMsg.left_arm_trajectory_message);
        //generate arm message
    }
    else
    {
        wholeBodyMsg.right_arm_trajectory_message.unique_id=wholeBodyMsg.unique_id;
        rightArmMsg(wholeBodyMsg,traj,joint_limits_right_);
    }

    // Chest
    chestMsg(wholeBodyMsg,traj);

    m_wholebodyPub.publish(wholeBodyMsg);
    ROS_INFO("Published whole body msg");
    ros::Duration(1).sleep();
}

void WholebodyControlInterface::leftArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj,std::vector<std::pair<double, double> > joint_limits_)
{
    msg.left_arm_trajectory_message.joint_trajectory_messages.resize(7);
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
            //            ihmc_pointMsg.position= ihmc_pointMsg.position <= joint_limits_[jointNumber-3].first  ? joint_limits_[jointNumber-3].first : ihmc_pointMsg.position;
            //            ihmc_pointMsg.position = ihmc_pointMsg.position>= joint_limits_[jointNumber-3].second ? joint_limits_[jointNumber-3].second : ihmc_pointMsg.position;
            ihmc_pointMsg.velocity = traj.points[trajPointNumber].velocities[jointNumber];

            msg.left_arm_trajectory_message.joint_trajectory_messages[jointNumber-3].trajectory_points.push_back(ihmc_pointMsg);
        }

    }
}

void WholebodyControlInterface::rightArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj, std::vector<std::pair<double, double> > joint_limits_)
{
    msg.right_arm_trajectory_message.joint_trajectory_messages.resize(7);
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
            //            ihmc_pointMsg.position= ihmc_pointMsg.position <= joint_limits_[jointNumber-3].first  ? joint_limits_[jointNumber-3].first : ihmc_pointMsg.position;
            //            ihmc_pointMsg.position = ihmc_pointMsg.position>= joint_limits_[jointNumber-3].second ? joint_limits_[jointNumber-3].second : ihmc_pointMsg.position;
            ihmc_pointMsg.velocity = traj.points[trajPointNumber].velocities[jointNumber];

            msg.right_arm_trajectory_message.joint_trajectory_messages[jointNumber-3].trajectory_points.push_back(ihmc_pointMsg);
        }

    }
}

void WholebodyControlInterface::chestMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj)
{
    geometry_msgs::Pose pelvisPose;
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

        ihmc_msgs::SO3TrajectoryPointRosMessage data;
        data.orientation = tempQuat.quaternion;
        data.time = traj.points[i].time_from_start.toSec();
        msg.chest_trajectory_message.taskspace_trajectory_points.push_back(data);
    }

}

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

WholebodyControlInterface::TrajectoryType WholebodyControlInterface::getTrajectoryType(const trajectory_msgs::JointTrajectory &traj)
{
    TrajectoryType type = TrajectoryType::INVALID;

    // size of position vector inside every trajectory point should be either 7 or 10.
    for (trajectory_msgs::JointTrajectoryPoint point : traj.points){
        if (point.positions.size() == 7 && (type == TrajectoryType::SEVEN_DOF || type == TrajectoryType::INVALID)){
            type =  TrajectoryType::SEVEN_DOF ;
        }
        else if (point.positions.size() == 10 && (type == TrajectoryType::TEN_DOF || type == TrajectoryType::INVALID)) {
            type =  WholebodyControlInterface::TrajectoryType::TEN_DOF ;
        }
        else {
            return TrajectoryType::INVALID;
        }
    }
    return type;
}

void WholebodyControlInterface::jointTrjectoryToArmMessage(const trajectory_msgs::JointTrajectory &traj, ihmc_msgs::ArmTrajectoryRosMessage &msg)
{
    
    // find left arm starting index
    std::vector<std::string> left_arm_joint_names;
    rd_->getLeftArmJointNames(left_arm_joint_names);
    

}

void WholebodyControlInterface::generateArmMessage(RobotSide side, const trajectory_msgs::JointTrajectory traj, const   std::vector<std::string> &left_arm_joint_names, ihmc_msgs::ArmTrajectoryRosMessage & msg)
{
    auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), left_arm_joint_names.at(0));
    int first = std::distance(traj.joint_names.begin(), it);
    if(it == traj.joint_names.end()){
        // left arm joints dont exist in the trajectory.
    }
    else {
        auto it_temp = it;
        for (auto arm_joint_it = left_arm_joint_names.begin(); arm_joint_it != left_arm_joint_names.end(); ++arm_joint_it, ++it_temp)
        {
            if(*it_temp != *arm_joint_it) {
                // joints in trajectory are not in the expected sequence.
            }
        }

        // joint names exist in trajectory and they are in expected sequence
        std::vector<std::vector<double> > arm_pose ;

        for(int index = first; index < left_arm_joint_names.size(); ++index)
        {
            // check size of positions here
            arm_pose.push_back(traj.points.at(index).positions);
        }
    }
}


