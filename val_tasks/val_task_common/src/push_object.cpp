#include <ros/ros.h>
#include <val_controllers/robot_state.h>
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_control_common/val_control_common.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "push_object1");
    ros::NodeHandle nh;

    geometry_msgs::QuaternionStamped leftHandOrientation_,rightHandOrientation_;
    leftHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientation_.quaternion.x = 0.492;
    leftHandOrientation_.quaternion.y = 0.504;
    leftHandOrientation_.quaternion.z = -0.494;
    leftHandOrientation_.quaternion.w = 0.509;

    /* Top Grip Flat Hand */
    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    rightHandOrientation_.quaternion.x = -0.549;
    rightHandOrientation_.quaternion.y = 0.591;
    rightHandOrientation_.quaternion.z = 0.560;
    rightHandOrientation_.quaternion.w = 0.188;

    cartesianPlanner* right_arm_planner;
    cartesianPlanner* left_arm_planner;
    armTrajectory *armTraj;
    chestTrajectory* chest_controller_;
    wholebodyManipulation* wholebody_controller_;
    valControlCommon* control_common_;
    RobotStateInformer *current_state_;

    wholebody_controller_ = new wholebodyManipulation(nh);
    armTraj               = new armTrajectory(nh);
    chest_controller_     = new chestTrajectory(nh);
    control_common_       = new valControlCommon(nh);
    right_arm_planner     = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    left_arm_planner      = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    current_state_ = RobotStateInformer::getRobotStateInformer(nh);

    armSide side;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose intermGoal1,intermGoal2;
    moveit_msgs::RobotTrajectory traj;

        if(argc == 8 )
        {
        side = argv[1][0] == '1'? armSide::RIGHT :armSide::LEFT;
        if(side == armSide::LEFT)
        {
            waypoints.clear();

            intermGoal1.position.x=std::atof(argv[2]);
            intermGoal1.position.x=std::atof(argv[3]);
            intermGoal1.position.x=std::atof(argv[4]);
            current_state_->transformQuaternion(leftHandOrientation_, leftHandOrientation_);
            intermGoal1.orientation=leftHandOrientation_.quaternion;
            waypoints.push_back(intermGoal1);

            armTraj->moveArmInTaskSpace(side, intermGoal1, 2.0f);
            ros::Duration(2).sleep();
            control_common_->stopAllTrajectories();

            current_state_->transformPose(intermGoal1,intermGoal2,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
            intermGoal2.position.x+=std::atof(argv[5]);
            intermGoal2.position.y+=std::atof(argv[6]);
            intermGoal2.position.z+=std::atof(argv[7]);
            current_state_->transformPose(intermGoal2,intermGoal2,VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF);
            waypoints.push_back(intermGoal2);

            left_arm_planner->getTrajFromCartPoints(waypoints, traj, false);

        }
        else
        {
            waypoints.clear();
            intermGoal1.position.x=std::atof(argv[2]);
            intermGoal1.position.x=std::atof(argv[3]);
            intermGoal1.position.x=std::atof(argv[4]);
            current_state_->transformQuaternion(rightHandOrientation_, rightHandOrientation_);
            intermGoal1.orientation=rightHandOrientation_.quaternion;
            waypoints.push_back(intermGoal1);

            armTraj->moveArmInTaskSpace(side, intermGoal1, 2.0f);
            ros::Duration(2).sleep();
            control_common_->stopAllTrajectories();

            current_state_->transformPose(intermGoal1,intermGoal2,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
            intermGoal2.position.x+=std::atof(argv[5]);
            intermGoal2.position.y+=std::atof(argv[6]);
            intermGoal2.position.z+=std::atof(argv[7]);
            current_state_->transformPose(intermGoal2,intermGoal2,VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF);
            waypoints.push_back(intermGoal2);

            right_arm_planner->getTrajFromCartPoints(waypoints, traj, false);
        }

        // Planning whole body motion
        wholebody_controller_->compileMsg(side,traj.joint_trajectory);
        ros::Duration(2).sleep();
    }
    else std::cout<<"invalid input \n";

    if(wholebody_controller_ != nullptr)      delete wholebody_controller_;
    if(armTraj != nullptr)                    delete armTraj;
    if(chest_controller_ != nullptr)          delete chest_controller_;
    if(control_common_ != nullptr)            delete control_common_;
    if(right_arm_planner != nullptr)          delete right_arm_planner;
    if(left_arm_planner != nullptr)           delete left_arm_planner;
}

