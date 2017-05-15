#include "val_control/val_wholebody_manipulation.h"

wholebodyManipulation::wholebodyManipulation(ros::NodeHandle &nh):nh_(nh)
{
    m_wholebodyPub = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 10, true);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

void wholebodyManipulation::compileMsg(const armSide side, const trajectory_msgs::JointTrajectory &traj)
{
    ihmc_msgs::WholeBodyTrajectoryRosMessage wholeBodyMsg;

    //Solving for side conflicts
    wholeBodyMsg.left_arm_trajectory_message.robot_side=0;
    wholeBodyMsg.right_arm_trajectory_message.robot_side=1;

    wholeBodyMsg.left_foot_trajectory_message.robot_side=0;
    wholeBodyMsg.right_foot_trajectory_message.robot_side=1;

    wholeBodyMsg.left_hand_trajectory_message.robot_side=0;
    wholeBodyMsg.right_hand_trajectory_message.robot_side=1;

    // Clearing trajectory points
    wholeBodyMsg.left_arm_trajectory_message.joint_trajectory_messages.clear();
    wholeBodyMsg.right_arm_trajectory_message.joint_trajectory_messages.clear();

    // Specifying execution modes
    wholeBodyMsg.chest_trajectory_message.execution_mode=wholeBodyMsg.chest_trajectory_message.OVERRIDE;
    wholeBodyMsg.right_arm_trajectory_message.execution_mode=wholeBodyMsg.right_arm_trajectory_message.OVERRIDE;
    wholeBodyMsg.left_arm_trajectory_message.execution_mode=wholeBodyMsg.left_arm_trajectory_message.OVERRIDE;

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

    // Arm
    if(side == LEFT)
    {
        wholeBodyMsg.left_arm_trajectory_message.unique_id=wholeBodyMsg.unique_id;
        leftArmMsg(wholeBodyMsg,traj);
    }
    else
    {
        wholeBodyMsg.right_arm_trajectory_message.unique_id=wholeBodyMsg.unique_id;
        rightArmMsg(wholeBodyMsg,traj);
    }

    // Chest
    chestMsg(wholeBodyMsg,traj);


    m_wholebodyPub.publish(wholeBodyMsg);
    ROS_INFO("Published whole body msg");
    ros::Duration(1).sleep();
}

void wholebodyManipulation::leftArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj)
{
    msg.left_arm_trajectory_message.joint_trajectory_messages.resize(7);
    for(int trajPointNumber = 0; trajPointNumber < traj.points.size(); trajPointNumber++){
        for (int jointNumber = 3; jointNumber < 10; ++jointNumber) {
            ihmc_msgs::TrajectoryPoint1DRosMessage ihmc_pointMsg;
            ihmc_pointMsg.time = traj.points[trajPointNumber].time_from_start.toSec();

            ihmc_pointMsg.position = traj.points[trajPointNumber].positions[jointNumber];
            ihmc_pointMsg.velocity = traj.points[trajPointNumber].velocities[jointNumber];

            msg.left_arm_trajectory_message.joint_trajectory_messages[jointNumber-3].trajectory_points.push_back(ihmc_pointMsg);
            //            ROS_INFO("point %d , joint %d , value %.4f", trajPointNumber, jointNumber, ihmc_pointMsg.position );
        }

    }
}

void wholebodyManipulation::rightArmMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj)
{
    msg.right_arm_trajectory_message.joint_trajectory_messages.resize(7);
    for(int trajPointNumber = 0; trajPointNumber < traj.points.size(); trajPointNumber++){
        for (int jointNumber = 3; jointNumber < 10; ++jointNumber) {
            ihmc_msgs::TrajectoryPoint1DRosMessage ihmc_pointMsg;
            ihmc_pointMsg.time = traj.points[trajPointNumber].time_from_start.toSec();

            ihmc_pointMsg.position = traj.points[trajPointNumber].positions[jointNumber];
            ihmc_pointMsg.velocity = traj.points[trajPointNumber].velocities[jointNumber];

            msg.right_arm_trajectory_message.joint_trajectory_messages[jointNumber-3].trajectory_points.push_back(ihmc_pointMsg);
            //            ROS_INFO("point %d , joint %d , value %.4f", trajPointNumber, jointNumber, ihmc_pointMsg.position );
        }

    }
}

void wholebodyManipulation::chestMsg(ihmc_msgs::WholeBodyTrajectoryRosMessage &msg, const trajectory_msgs::JointTrajectory &traj)
{
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,pelvisPose);
    for (int i = 0; i < traj.points.size(); ++i) {
        float yaw   = traj.points[i].positions[0];
        float pitch = traj.points[i].positions[1];
        float roll  = traj.points[i].positions[2];

        tf::Quaternion quatPelvis;
        quatPelvis.setRPY(roll, pitch, yaw);
        geometry_msgs::QuaternionStamped tempQuat;
        tempQuat.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
        tf::quaternionTFToMsg(quatPelvis, tempQuat.quaternion);

        robot_state_->transformQuaternion(tempQuat, tempQuat);

        ihmc_msgs::SO3TrajectoryPointRosMessage data;
        data.orientation = tempQuat.quaternion;
        data.time = traj.points[i].time_from_start.toSec();
        msg.chest_trajectory_message.taskspace_trajectory_points.push_back(data);
    }

}

