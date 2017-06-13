#include <val_task3/leak_detector_grabber.h>
#include <stdlib.h>
#include <stdio.h>
#include <eigen_conversions/eigen_msg.h>

// Detector height relative to floor (not world frame, which is untrustworthy)
#define DETECTOR_GRASP_HEIGHT 0.92

#define INTERMEDIATE_GOAL_OFFSET 0.2

#define PREGRASP_LEFT   {1.3999, -0.3, 0.0, 0.0, 0.0}
#define PREGRASP_RIGHT  PREGRASP_LEFT

#define GRASP_LEFT      {1.3999, -0.55, -1.1, -0.9, -1.0}
#define GRASP_RIGHT     {1.3999, -0.55, 1.1, 0.9, 1.0}

leakDetectorGrabber::leakDetectorGrabber(ros::NodeHandle nh):nh_(nh),
    armTraj_(nh_), gripper_(nh_), wholebody_controller_(nh_), task3_utils_(nh_)
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 30, false);
    right_arm_planner_ = new cartesianPlanner("rightPalm");
    left_arm_planner_ = new cartesianPlanner("leftPalm");
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

leakDetectorGrabber::~leakDetectorGrabber(){

}

geometry_msgs::Pose leakDetectorGrabber::getGraspGoal(const armSide &side, const geometry_msgs::Pose &user_goal) const {
    std::string palm_frame, end_effector_frame, thumb_frame;
    double pregrasp_angle;
    if(side == armSide::LEFT){
        palm_frame = VAL_COMMON_NAMES::L_PALM_TF;
        end_effector_frame = VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME;
        thumb_frame = "/leftThumbRollLink";
        pregrasp_angle = -0.6977;
    } else {
        palm_frame = VAL_COMMON_NAMES::R_PALM_TF;
        end_effector_frame = VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME;
        thumb_frame = "/rightThumbRollLink";
        pregrasp_angle = 0.6977;
    }

    geometry_msgs::Pose grasp_goal;

    geometry_msgs::Pose thumb_offset;
    current_state_->getCurrentPose(palm_frame, thumb_offset, thumb_frame);
    current_state_->transformPose(user_goal, grasp_goal, VAL_COMMON_NAMES::WORLD_TF, palm_frame);
    grasp_goal.position.x += thumb_offset.position.x;
    grasp_goal.position.y += thumb_offset.position.y;
    grasp_goal.position.z += thumb_offset.position.z;
    current_state_->transformPose(grasp_goal, grasp_goal, palm_frame, VAL_COMMON_NAMES::WORLD_TF);

    taskCommonUtils::fixHandFramePalmDown(nh_, side, grasp_goal);

    // Rotate goal by -0.6977 about z -- this is the wrist angle that causes the fingertips and the thumb to be at
    // the same height when the hand is in the pregrasp pose
    Eigen::Affine3d grasp_goal_eigen;
    tf::poseMsgToEigen(grasp_goal, grasp_goal_eigen);
    grasp_goal_eigen.rotate(Eigen::AngleAxisd(pregrasp_angle, Eigen::Vector3d::UnitZ()));
    tf::poseEigenToMsg(grasp_goal_eigen, grasp_goal);

    return grasp_goal;
}

geometry_msgs::Pose leakDetectorGrabber::getReachGoal(const armSide &side, const geometry_msgs::Pose &grasp_goal) const {
    // Reach goal is the same pose as grasp goal except higher above the table
    geometry_msgs::Pose reach_goal;

    current_state_->transformPose(grasp_goal, reach_goal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    reach_goal.position.z += INTERMEDIATE_GOAL_OFFSET;

    //transform that point back to world frame
    current_state_->transformPose(reach_goal, reach_goal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

    return reach_goal;
}

float leakDetectorGrabber::getStandingOffset(const armSide side, const geometry_msgs::Pose user_goal) const {
    std::string shoulder_frame = (side == armSide::LEFT) ? "/leftShoulderRollLink" : "/rightShoulderRollLink";

    geometry_msgs::Pose goal_wrt_pelvis, shoulder_wrt_pelvis;
    current_state_->transformPose(user_goal, goal_wrt_pelvis, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    current_state_->getCurrentPose(shoulder_frame, shoulder_wrt_pelvis, VAL_COMMON_NAMES::PELVIS_TF);

    double yaw = tf::getYaw(goal_wrt_pelvis.orientation);

    // 0.356 is the approximate length of the forearm
    double delta_y = 0.356 * std::cos(yaw) * (side == armSide::LEFT ? 1 : -1);

    geometry_msgs::Pose standing_pose;
    standing_pose.position.y = goal_wrt_pelvis.position.y - shoulder_wrt_pelvis.position.y + delta_y;
    standing_pose.orientation.w = 1;
    current_state_->transformPose(standing_pose, standing_pose, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

    pubPoseArrow(standing_pose, "detector_standing_pose", 1.f, 0.f, 0.f);

    return standing_pose.position.y;
}

// note that `side` is ignored if GRAB_WITH_EITHER_HAND is true
void leakDetectorGrabber::graspDetector(geometry_msgs::Pose user_goal, float executionTime) {
    // If user_goal's z is exactly zero, it's almost certainly generated from Rviz and needs its z determined
    if (user_goal.position.z == 0) {
        geometry_msgs::Pose foot_bottom_world_frame;
        current_state_->getCurrentPose("/leftCOP_Frame", foot_bottom_world_frame, VAL_COMMON_NAMES::WORLD_TF);
        user_goal.position.z = foot_bottom_world_frame.position.z + DETECTOR_GRASP_HEIGHT;

        ROS_DEBUG_STREAM("Goal pose z determined to be " << user_goal.position.z);
    }

    armSide side;
    // this overwrites both side and user_goal
    user_goal = task3_utils_.grasping_hand(side, user_goal);
    ROS_DEBUG_STREAM("Grasping with " << (side == armSide::LEFT ? "left" : "right") << " hand");

    ROS_INFO("Moving hand to pre-grasp");
    if (side == armSide::LEFT) {
        gripper_.controlGripper(side, PREGRASP_LEFT);
    } else {
        gripper_.controlGripper(side, PREGRASP_RIGHT);
    }

    float standing_offset = getStandingOffset(side, user_goal);

    ROS_INFO_STREAM("Walking to standing pose (TODO) (offset " << standing_offset << ")");
    ros::Duration(2).sleep();

    //move arm to given point with known orientation and higher z
    geometry_msgs::Pose grasp_goal = getGraspGoal(side, user_goal);
    geometry_msgs::Pose reach_goal = getReachGoal(side, grasp_goal);

    pubPoseArrow(user_goal, "detector_user_goal", 1.f, 1.f, 0.f);
    pubPoseArrow(reach_goal, "detector_reach_goal", 1.f, 0.f, 1.f);
    pubPoseArrow(grasp_goal, "detector_grasp_goal", 0.f, 1.f, 0.f);

    ROS_INFO("Moving to reach goal (reaches towards a point above the detector with the arm only)");

    armTraj_.moveArmInTaskSpace(side, reach_goal, executionTime * 2);
    ros::Duration(executionTime * 2).sleep();

    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(reach_goal);
    waypoints.push_back(grasp_goal);

    moveit_msgs::RobotTrajectory traj;

//    ROS_DEBUG("Debug: delaying before actual grab");
//    ros::Duration(5).sleep();

    ROS_INFO("Calculating trajectory for grasp");
    if(side == armSide::LEFT) {
        left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    } else{
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
    }

    ROS_INFO("Executing grasp trajectory");
    wholebody_controller_.compileMsg(side, traj.joint_trajectory);

    ros::Duration(executionTime + 0.5).sleep();

    ROS_INFO("Closing gripper");
    if (side == armSide::LEFT) {
        taskCommonUtils::slowGrip(nh_, side, PREGRASP_LEFT, GRASP_LEFT);
    } else {
        taskCommonUtils::slowGrip(nh_, side, PREGRASP_RIGHT, GRASP_RIGHT);
    }
    ros::Duration(0.3).sleep();
}

void leakDetectorGrabber::pubPoseArrow(const geometry_msgs::Pose &pose_msg, const std::string &ns, const int id,
                                       const float r, const float g, const float b) const {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(pose_msg, pose);

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "world";
    marker.ns = ns;
    marker.id = id * 10;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.scale.x = 0.02; // Shaft diameter
    marker.scale.y = 0.05;  // Head diameter
    marker.scale.z = 0;    // Head length, or 0 for default
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1;

    marker.points.resize(2);
    // Start point: d distance along the normal direction
    marker.points.front().x = pose.translation()[0];
    marker.points.front().y = pose.translation()[1];
    marker.points.front().z = pose.translation()[2];
    // End point: d + 0.1 distance along the normal direction
    auto unitx = pose * Eigen::Vector3d(0.25, 0, 0);
    marker.points.back().x = unitx[0];
    marker.points.back().y = unitx[1];
    marker.points.back().z = unitx[2];

    ma.markers.push_back(marker);

    marker.scale.x /= 2; // Shaft diameter
    marker.scale.y /= 2;  // Head diameter
    marker.scale.z /= 2;    // Head length, or 0 for default
    marker.id = id * 10 + 1;
    auto unitz = pose * Eigen::Vector3d(0, 0, 0.125);
    marker.points.back().x = unitz[0];
    marker.points.back().y = unitz[1];
    marker.points.back().z = unitz[2];

    ma.markers.push_back(marker);

    marker.id = id * 10 + 2;
    auto unity = pose * Eigen::Vector3d(0, 0.125, 0);
    marker.points.back().x = unity[0];
    marker.points.back().y = unity[1];
    marker.points.back().z = unity[2];

    ma.markers.push_back(marker);

    marker_pub_.publish(ma);
}
