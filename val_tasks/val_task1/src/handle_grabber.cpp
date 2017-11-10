#include <val_task1/handle_grabber.h>
#include <stdlib.h>
#include <stdio.h>


handle_grabber::handle_grabber(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    leftHandOrientation_.header.frame_id = rd_->getPelvisFrame();
    /* Top Grip */

    leftHandOrientation_.quaternion.x = 0.604;
    leftHandOrientation_.quaternion.y = 0.434;
    leftHandOrientation_.quaternion.z = -0.583;
    leftHandOrientation_.quaternion.w = 0.326;

    //    /* Side Grip */
    //    leftHandOrientation_.quaternion.x = 0.155;
    //    leftHandOrientation_.quaternion.y = -0.061;
    //    leftHandOrientation_.quaternion.z = -0.696;
    //    leftHandOrientation_.quaternion.w = 0.699;

    //    /* Side Grip */
    //    rightHandOrientation_.header.frame_id = VAL_COMMON_NAMES::PELVIS_TF;
    //    rightHandOrientation_.quaternion.x = -0.094;
    //    rightHandOrientation_.quaternion.y = -0.027;
    //    rightHandOrientation_.quaternion.z = 0.973;
    //    rightHandOrientation_.quaternion.w = -0.209;

    /* Top Grip */
    rightHandOrientation_.header.frame_id = rd_->getPelvisFrame();
    rightHandOrientation_.quaternion.x = -0.576;
    rightHandOrientation_.quaternion.y = 0.397;
    rightHandOrientation_.quaternion.z = 0.632;
    rightHandOrientation_.quaternion.w = 0.332;

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftPalm", VAL_COMMON_NAMES::WORLD_TF); //leftMiddleFingerGroup
    right_arm_planner_ = new cartesianPlanner("rightPalm", VAL_COMMON_NAMES::WORLD_TF); //rightMiddleFingerGroup
    wholebody_controller_ = new wholebodyManipulation(nh_);
}

handle_grabber::~handle_grabber()
{
    delete left_arm_planner_;
    delete right_arm_planner_;
    delete wholebody_controller_;
}


geometry_msgs::QuaternionStamped handle_grabber::leftHandOrientation() const
{
    return leftHandOrientation_;
}

void handle_grabber::setLeftHandOrientation(const geometry_msgs::QuaternionStamped &leftHandOrientation)
{
    leftHandOrientation_ = leftHandOrientation;
}
geometry_msgs::QuaternionStamped handle_grabber::rightHandOrientation() const
{
    return rightHandOrientation_;
}

void handle_grabber::setRightHandOrientation(const geometry_msgs::QuaternionStamped &rightHandOrientation)
{
    rightHandOrientation_ = rightHandOrientation;
}


void handle_grabber::adjust_pose(const armSide side, const geometry_msgs::Point &goal, float executionTime){

    //move arm to given point with known orientation and higher z
    ROS_INFO("Moving at an intermidate point before goal");
    geometry_msgs::Pose pt;
    pt.position = goal;
    geometry_msgs::QuaternionStamped temp  = side == armSide::LEFT ? leftHandOrientation_ : rightHandOrientation_;
    current_state_->transformQuaternion(temp, temp);

    pt.orientation = temp.quaternion;

    geometry_msgs::Pose pose;
    std::string fingerFrame = side == LEFT ? rd_->getLeftEEFrame() : rd_->getRightEEFrame();
    std::string palmFrame = side == LEFT ? rd_->getLeftPalmFrame() : rd_->getRightPalmFrame();

    current_state_->getCurrentPose(fingerFrame, pose, palmFrame);
    if(side == armSide::LEFT){
        pt.position.x -= pose.position.x;
        pt.position.y -= pose.position.y;
        pt.position.z -= pose.position.z;
        pt.position.z -= 0.1;
    }
    else{
        pt.position.x -= pose.position.x;
        pt.position.y += pose.position.y;
        pt.position.z -= pose.position.z;
        pt.position.z -= 0.1;
    }
    ROS_INFO("Nudge forward to align palm");
    armTraj_.moveArmInTaskSpace(side, pt, executionTime);
    ros::Duration(executionTime*1.4).sleep();

}

void handle_grabber::grasp_handles(const armSide side, const geometry_msgs::Point &goal, float executionTime)
{
    const std::vector<float>* seed;
    geometry_msgs::QuaternionStamped* finalOrientationStamped;
    if(side == armSide::LEFT){
        seed = &leftShoulderSeed_;
        finalOrientationStamped = &leftHandOrientation_;
    }
    else {
        seed = &rightShoulderSeed_;
        finalOrientationStamped = &rightHandOrientation_;
    }


    ROS_INFO("opening grippers");
    gripper_.controlGripper(side, GRIPPER_STATE::OPEN);

    //move shoulder roll outwards
    ROS_INFO("Setting shoulder roll");
    std::vector< std::vector<float> > armData;
    armData.push_back(*seed);

    armTraj_.moveArmJoints(side, armData, executionTime);
    ros::Duration(executionTime*2).sleep();


    //move arm to given point with known orientation and higher z
    geometry_msgs::Pose finalGoal, intermGoal;
    geometry_msgs::Point finalPoint, intermPoint;

    current_state_->transformPoint(goal,intermPoint, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
    intermPoint.z += 0.1;

    //transform that point back to world frame
    current_state_->transformPoint(intermPoint, intermPoint, rd_->getPelvisFrame(), VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving at an intermidate point before goal");
    intermGoal.position = intermPoint;
    geometry_msgs::QuaternionStamped temp  = *finalOrientationStamped;

    current_state_->transformQuaternion(temp, temp);
    intermGoal.orientation = temp.quaternion;

    armTraj_.moveArmInTaskSpace(side, intermGoal, executionTime);
    ros::Duration(executionTime*2).sleep();

    //move arm to final position with known orientation

    current_state_->transformPoint(goal,finalPoint, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
    finalPoint.x -= 0.05; // this is to compensate for the distance between palm frame and center of palm
//    finalPoint.y = side == armSide::LEFT ? (finalPoint.y-0.02) : (finalPoint.y+0.02);

    //transform that point back to world frame
    current_state_->transformPoint(finalPoint, finalPoint, rd_->getPelvisFrame(), VAL_COMMON_NAMES::WORLD_TF);

    ROS_INFO("Moving towards goal");
    finalGoal.position = finalPoint;
    finalGoal.orientation = temp.quaternion;

    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(finalGoal);

    moveit_msgs::RobotTrajectory traj;
    if (side == armSide::RIGHT) right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    if (side == armSide::LEFT)   left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);

    wholebody_controller_->compileMsg(side, traj.joint_trajectory);

    ros::Duration(executionTime).sleep();
    ROS_INFO("Closing grippers");
    gripper_.controlGripper(side, GRIPPER_STATE::HANDLE_HOLD);
    ros::Duration(0.6).sleep();
}
