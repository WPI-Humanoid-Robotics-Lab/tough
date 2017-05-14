#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "val_task1/val_task1.h"
#include "navigation_common/map_generator.h"
#include "srcsim/StartTask.h"
#include <val_control/robot_state.h>

#define foreach BOOST_FOREACH

valTask1 *valTask1::currentObject = nullptr;

valTask1* valTask1::getValTask1(ros::NodeHandle nh){
    if(currentObject == nullptr){
        currentObject = new valTask1(nh);
        return currentObject;
    }
    ROS_ERROR("Object already exists");
    assert(false && "Object already exists");
}

// constructor and destrcutor
valTask1::valTask1(ros::NodeHandle nh):
    nh_(nh)
{
    // object for the valkyrie walker
    walker_ = new ValkyrieWalker(nh_, 0.7, 0.7, 0, 0.18);

    // object for tracking walk
    walk_track_ = new walkTracking(nh);

    // panel detection
    panel_detector_     = nullptr;
    finish_box_detector_= nullptr;
    handle_detector_    = new HandleDetector(nh_);
    handle_grabber_     = new handle_grabber(nh_);
    move_handle_        = new move_handle(nh_);

    // controllers
    chest_controller_   = new chestTrajectory(nh_);
    pelvis_controller_  = new pelvisTrajectory(nh_);
    head_controller_    = new HeadTrajectory(nh_);
    gripper_controller_ = new gripperControl(nh_);
    arm_controller_     = new armTrajectory(nh_);

    //state informer
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    map_update_count_ = 0;
    occupancy_grid_sub_ = nh_.subscribe("/map",10, &valTask1::occupancy_grid_cb, this);
    visited_map_sub_    = nh_.subscribe("/visited_map",10, &valTask1::visited_map_cb, this);

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftArm", "/world");
    right_arm_planner_ = new cartesianPlanner("rightArm", "/world");

    // task1 utils
    task1_utils_ = new task1Utils(nh_);
}

// destructor
valTask1::~valTask1(){

    if(walker_ != nullptr)              delete walker_;
    if(walk_track_ != nullptr)          delete walk_track_;
    if(panel_detector_ != nullptr)      delete panel_detector_;
    if(handle_detector_ != nullptr)     delete handle_detector_;
    if(move_handle_ != nullptr)         delete move_handle_;
    if(chest_controller_ != nullptr)    delete chest_controller_;
    if(pelvis_controller_ != nullptr)   delete pelvis_controller_;
    if(head_controller_ != nullptr)     delete head_controller_;
    if(gripper_controller_ != nullptr)  delete gripper_controller_;
    if(robot_state_ != nullptr)         delete robot_state_;
    if(arm_controller_ != nullptr)      delete arm_controller_;
}

void valTask1::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){
    ++map_update_count_;
}

void valTask1::visited_map_cb(const nav_msgs::OccupancyGrid::Ptr msg)
{
    visited_map_ = *msg;
}

bool valTask1::preemptiveWait(double ms, decision_making::EventQueue& queue) {
    for (int i = 0; i < 100 && !queue.isTerminated(); i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

    return queue.isTerminated();
}

// state machine state executions
decision_making::TaskResult valTask1::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    static int retry_count = 0;

    // TODO
    // if the map does not update fast enought and this is called greater then 10 time it will break
    if(retry_count == 0){
        map_update_count_ = 0;
    }
    // It is depenent on the timer timer right now.

    // the state transition can happen from an event externally or can be geenerated here
    ROS_INFO("Occupancy Grid has been updated %d times, tried %d times", map_update_count_, retry_count);
    if (map_update_count_ > 1) {
        // move to a configuration that is robust while walking
        retry_count = 0;
        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(1.0f).sleep();

        // start the task
        ros::ServiceClient  client = nh_.serviceClient<srcsim::StartTask>("/srcsim/finals/start_task");
        srcsim::StartTask   srv;
        srv.request.checkpoint_id = 1;
        srv.request.task_id       = 1;
        if(client.call(srv)) {
            //what do we do if this call fails or succeeds?
        }
        // generate the event
        eventQueue.riseEvent("/INIT_SUCESSFUL");

    }
    else if (map_update_count_ < 2 && retry_count++ < 40) {
        ROS_INFO("Wait for occupancy grid to be updated with atleast 2 messages");
        ros::Duration(2.0).sleep();
        eventQueue.riseEvent("/INIT_RETRY");
    }
    else {
        retry_count = 0;
        ROS_INFO("Failed to initialize");
        eventQueue.riseEvent("/INIT_FAILED");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectPanelCoarseTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("executing " << name);

    if(panel_detector_ == nullptr) {
        panel_detector_ = new PanelDetector(nh_, DETECTOR_TYPE::HANDLE_PANEL_COARSE);
    }

    static int fail_count = 0;
    static int retry_count = 0;

    // detect panel
    std::vector<geometry_msgs::Pose> poses;
    panel_detector_->getDetections(poses);
    ROS_INFO("Size of poses : %d", (int)poses.size());
    // if we get atleast one detection, LOL
    if (poses.size() > 1)
    {
        // update the pose
        geometry_msgs::Pose2D pose2D;
        // get the last detected pose
        int idx = poses.size() -1 ;
        pose2D.x = poses[idx].position.x;
        pose2D.y = poses[idx].position.y;

        std::cout << "x " << pose2D.x << " y " << pose2D.y << std::endl;

        // get the theta
        pose2D.theta = tf::getYaw(poses[idx].orientation);
        setPanelWalkGoal(pose2D);

        std::cout << "quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w <<std::endl;
        std::cout << "yaw: " << pose2D.theta  <<std::endl;
        retry_count = 0;
        // update the plane coeffecients
        if(panel_detector_->getPanelPlaneModel(panel_coeff_)){
            eventQueue.riseEvent("/DETECTED_PANEL");
            ROS_INFO("detected panel");
            if(panel_detector_ != nullptr) delete panel_detector_;
            ROS_INFO("I'm still alive");
            panel_detector_ = nullptr;
        }
        else{
            ROS_INFO("Could not query plane equation");
            ++retry_count;
            eventQueue.riseEvent("/DETECT_PANEL_RETRY");
        }
    }
    else if(retry_count < 5) {
        ROS_INFO("sleep for 3 seconds for panel detection");
        ++retry_count;
        ros::Duration(1).sleep();
        eventQueue.riseEvent("/DETECT_PANEL_RETRY");
    }

    // if failed for more than 5 times, go to error state


    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        eventQueue.riseEvent("/DETECT_PANEL_FAILED");
        if(panel_detector_ != nullptr) delete panel_detector_;
        panel_detector_ = nullptr;
    }
    // if failed retry detecting the panel
    else
    {
        // increment the fail count
        fail_count++;
        eventQueue.riseEvent("/DETECT_PANEL_RETRY");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}


decision_making::TaskResult valTask1::walkToSeePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM_ONCE("executing " << name);

    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    // Check if goal is reached before walking
    if (taskCommonUtils::isGoalReached(current_pelvis_pose, panel_walk_goal_coarse_))
    {
        ROS_INFO("reached panel");

        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/REACHED_PANEL");
        // required for robot to stablize as goal tolerance is high
        ros::Duration(1).sleep();
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, panel_walk_goal_coarse_))
    {
        ROS_INFO_STREAM("pose changed to "<<panel_walk_goal_coarse_);
        walker_->walkToGoal(panel_walk_goal_coarse_, false);
        // sleep so that the walk starts
        ROS_INFO("Footsteps should be generated now");
        ros::Duration(4).sleep();

        // update the previous pose
        pose_prev = panel_walk_goal_coarse_;
        eventQueue.riseEvent("/WALK_EXECUTING");
    }

    // if walking stay in the same state
    else if (walk_track_->isWalking())
    {
        // no state change
        ROS_INFO_THROTTLE(2, "walking");
        eventQueue.riseEvent("/WALK_EXECUTING");
    }
    // if walk finished
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        ROS_INFO("walk failed");
        eventQueue.riseEvent("/WALK_FAILED");
    }
    // if failed retry detecting the panel and then walk
    // also handles MOVE_FAILED
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("walk retry");
        eventQueue.riseEvent("/WALK_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectHandleCenterTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    //tilt head downwards to see the panel
    head_controller_->moveHead(0.0f, 30.0f, 0.0f, 2.0f);
    static int retry_count = 0;
    //wait for head to be in position
    ros::Duration(3).sleep();

    //detect handles
    if( handle_detector_->findHandles(handle_loc_)){

        ROS_INFO_STREAM("Handles detected at "<<handle_loc_[0]<< " : "<<handle_loc_[2]);

        // generate the event
        eventQueue.riseEvent("/DETECTED_HANDLE");
    }
    else if( retry_count++ < 10){
        ROS_INFO("Did not detect handle, retrying");
        eventQueue.riseEvent("/DETECT_HANDLE_RETRY");
    }
    else{
        ROS_INFO("Did not detect handle, failed");
        eventQueue.riseEvent("/DETECT_HANDLE_FAILED");
        retry_count = 0;
    }


    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectPanelFineTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("executing " << name);

    if(panel_detector_ == nullptr) {
        panel_detector_ = new PanelDetector(nh_, DETECTOR_TYPE::HANDLE_PANEL_FINE);
        ros::Duration(0.2).sleep();
    }

    static int fail_count = 0;
    static int retry_count = 0;

    // detect panel
    std::vector<geometry_msgs::Pose> poses;
    panel_detector_->getDetections(poses);

    // if we get atleast one detection
    if (poses.size() > 1)
    {
        // update the pose
        geometry_msgs::Pose2D pose2D;
        // get the last detected pose
        int idx = poses.size() -1 ;
        pose2D.x = poses[idx].position.x;
        pose2D.y = poses[idx].position.y;

        std::cout << "x " << pose2D.x << " y " << pose2D.y << std::endl;

        // get the theta
        pose2D.theta = tf::getYaw(poses[idx].orientation);
        setPanelWalkGoalFine(pose2D);

        std::cout << "quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w <<std::endl;
        std::cout << "yaw: " << pose2D.theta  <<std::endl;
        retry_count = 0;
        // update the plane coeffecients
        if(panel_detector_->getPanelPlaneModel(panel_coeff_)){
            eventQueue.riseEvent("/DETECTED_PANEL_FINE");
            if(panel_detector_ != nullptr) delete panel_detector_;
            panel_detector_ = nullptr;
        }
        else{
            ROS_INFO("Could not query plane equation");
            ++retry_count;
            eventQueue.riseEvent("/DETECT_PANEL_FINE_RETRY");
        }
    }

    else if(retry_count < 5) {
        ROS_INFO("sleep for 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_PANEL_FINE_RETRY");
        ros::Duration(3).sleep();
    }

    // if failed for more than 5 times, go to error state


    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        eventQueue.riseEvent("/DETECT_PANEL_FINE_FAILED");
        if(panel_detector_ != nullptr) delete panel_detector_;
        panel_detector_ = nullptr;
    }
    // if failed retry detecting the panel
    else
    {
        // increment the fail count
        fail_count++;
        eventQueue.riseEvent("/DETECT_PANEL_FINE_RETRY");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::walkToPanel(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM_ONCE("executing " << name);

    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    // check if the pose is changed
    if (taskCommonUtils::isGoalReached(current_pelvis_pose, panel_walk_goal_fine_) )
    {

        ROS_INFO("reached panel");
        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/REACHED_PANEL_FINE");
        // required for robot to stablize as goal tolerance is high
        ros::Duration(1).sleep();
    }
    else if (taskCommonUtils::isPoseChanged(pose_prev, panel_walk_goal_fine_))
    {
        ROS_INFO("pose changed");
        //reset chest before moving close to panel
        chest_controller_->controlChest(0, 0, 0);
        ROS_INFO("Adjusted Chest");
        walker_->walkToGoal(panel_walk_goal_fine_, false);
        ROS_INFO("Footsteps should be published now");
        // sleep so that the walk starts
        ros::Duration(4).sleep();

        // update the previous pose
        pose_prev = panel_walk_goal_fine_;
        eventQueue.riseEvent("/WALK_TO_PANEL_EXECUTING");
    }
    // if walking stay in the same state
    else if (walk_track_->isWalking())
    {
        // no state change
        ROS_INFO_THROTTLE(2, "walking");
        eventQueue.riseEvent("/WALK_TO_PANEL_EXECUTING");
    }
    // if walk finished
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        ROS_INFO("walk failed");
        eventQueue.riseEvent("/WALK_TO_PANEL_FAILED");
    }
    // if failed retry detecting the panel and then walk
    // also handles MOVE_FAILED
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("walk retry");
        eventQueue.riseEvent("/WALK_TO_PANEL_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask1::graspPitchHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    /*
     * Executing -> when grasp_handles is called
     * Retry -> grasp handles is called but it failed
     * Failed -> retry failed 5 times
     * Success -> grasp is successful
     */
    static bool executing = false;
    static int retry_count = 0;

    if(!executing){
        ROS_INFO("Executing the grasp handle command");
        executing = true;
        // move the chest, so we get maximum manipulability
        chest_controller_->controlChest(0, 10, 40);

        // grasp the handle
        //1 - right
        //3 - left
        // sleep before grasping (to account for sway coming with ches motion)
        ros::Duration(1).sleep();
        geometry_msgs::Pose pose;
        robot_state_->getCurrentPose(VAL_COMMON_NAMES::R_PALM_TF, pose);
        handle_grabber_->grasp_handles(armSide::RIGHT , handle_loc_[1]);
        eventQueue.riseEvent("/GRASP_PITCH_HANDLE_EXECUTING");
    }
    else if (robot_state_->isGraspped(armSide::RIGHT)){
        ROS_INFO("Grasp is successful");
        eventQueue.riseEvent("/GRASPED_PITCH_HANDLE");
    }
    else if (retry_count < 5 ){
        ROS_INFO("Grasp Failed, retrying");
        executing = false;
        ++retry_count;
        eventQueue.riseEvent("/GRASP_PITCH_HANDLE_RETRY");
    }
    else {
        ROS_INFO("Failed all conditions. state error");
        eventQueue.riseEvent("/GRASP_PITCH_HANDLE_FAILED");
    }

    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // reduce the pelvis height, so we get maximum maipulability
    ros::Duration(1).sleep();
    pelvis_controller_->controlPelvisHeight(0.8);

    // generate the way points in cartersian space
    std::vector<geometry_msgs::Pose> waypoints;
    //desired pose (i.e. the grab pose)
    geometry_msgs::Pose grab_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME,grab_pose);
    task1_utils_->getCircle3D(handle_loc_[0], handle_loc_[1], grab_pose, panel_coeff_, waypoints, 0.125, 10);

    ///@todo: remove the visulaisation
    task1_utils_->visulatise6DPoints(waypoints);

    // plan the trajectory
    moveit_msgs::RobotTrajectory traj;
    right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);

    // execute the trajectory
    arm_controller_->moveArmTrajectory(armSide::RIGHT, traj.joint_trajectory);

    //eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::graspYawHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    ///@todo: complete the state
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("executing " << name);

    //    geometry_msgs::Pose pose;
    //    robot_state_->getCurrentPose("/leftPalm", pose);
    //    handle_grabber_->grasp_handles(armSide::LEFT, handle_loc_[3]);
    //    ros::Duration(1).sleep();

    //    std::vector<geometry_msgs::Pose> leftWaypoints;
    //    if(move_handle_ == nullptr) {
    //        move_handle_ = new move_handle(nh_);
    //    }

    //    ros::Duration(1).sleep();
    //    move_handle_->createCircle(handle_loc_[2], 0, panel_coeff_, leftWaypoints);

    //    if(move_handle_ != nullptr) delete move_handle_;
    //    move_handle_ = nullptr;

    eventQueue.riseEvent("/YAW_CORRECTION_SUCESSFUL");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::redetectHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    ///@todo: complete the state

    return TaskResult::SUCCESS();
}


decision_making::TaskResult valTask1::detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    static int retry_count = 0;
    // generate the event
    if (finish_box_detector_ == nullptr){
        finish_box_detector_ = new FinishBoxDetector(nh_);
        ros::Duration(2).sleep();
    }

    std::vector<geometry_msgs::Point> detections;
    if(visited_map_.data.empty()){
        retry_count++;
        ROS_INFO("visited map is empty");
        ros::Duration(3).sleep();
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    } else  if(finish_box_detector_->getFinishBoxCenters(detections)){
        for(size_t i = 0; i < detections.size(); ++i){
            size_t index = MapGenerator::getIndex(detections[i].x, detections[i].y);
            ROS_INFO("Index in map %d and size of visited map is %d", (int)index, (int)visited_map_.data.size());
            if(visited_map_.data.at(index) == 50){
                continue;
            }

            next_finishbox_center_.x = detections[i].x;
            next_finishbox_center_.y = detections[i].y;
            next_finishbox_center_.theta = atan2(next_finishbox_center_.y, next_finishbox_center_.x);
            ROS_INFO("Successful");
            eventQueue.riseEvent("/DETECT_FINISH_SUCESSFUL");
            if(finish_box_detector_ != nullptr) delete finish_box_detector_;
            finish_box_detector_ = nullptr;
            //sleep is required to avoid moving to next state before subscriber is shutdown
            ros::Duration(2).sleep();
            break;
        }
        // this is to avoid detecting points that will always be in collision
        retry_count++;
    }
    else if(retry_count++ < 5){
        ros::Duration(3).sleep();
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    }
    else{
        eventQueue.riseEvent("/DETECT_FINISH_FAILED");
        if(finish_box_detector_ != nullptr) delete finish_box_detector_;
        finish_box_detector_ = nullptr;
        //sleep is required to avoid moving to next state before subscriber is shutdown
        ros::Duration(2).sleep();
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    static int fail_count = 0;

    // walk to the goal location

    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    // check if the pose is changed
    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, next_finishbox_center_)) {
        ROS_INFO("reached panel");
        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/WALK_TO_FINISH_SUCESSFUL");
    }
    // the goal can be updated on the run time
    else if (taskCommonUtils::isPoseChanged(pose_prev, next_finishbox_center_))
    {
        ROS_INFO("pose changed");
        //reset chest before moving close to panel
        chest_controller_->controlChest(0, 0, 0);
        ROS_INFO("Adjusted Chest");
        walker_->walkToGoal(next_finishbox_center_, false);
        ROS_INFO("Footsteps should be published now");
        // sleep so that the walk starts
        ros::Duration(2).sleep();

        // update the previous pose
        pose_prev = next_finishbox_center_;
        eventQueue.riseEvent("/WALK_TO_FINISH_EXECUTING");
    }

    // if walking stay in the same state
    else if (walk_track_->isWalking())
    {
        // no state change
        ROS_INFO_THROTTLE(2, "walking");
        eventQueue.riseEvent("/WALK_TO_FINISH_EXECUTING");
    }
    // if walk finished
    // TODO change to see if we are at the goal
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        ROS_INFO("walk failed");
        eventQueue.riseEvent("/WALK_TO_FINISH_ERROR");
    }
    // if failed retry detecting the panel and then walk
    // also handles MOVE_FAILED
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("walk retry");
        eventQueue.riseEvent("/WALK_TO_FINISH_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

// setter and getter methods
geometry_msgs::Pose2D valTask1::getPanelWalkGoal()
{
    return panel_walk_goal_coarse_;
}

void valTask1::setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    panel_walk_goal_coarse_ = panel_walk_goal;
}

void valTask1::setPanelWalkGoalFine(const geometry_msgs::Pose2D &panel_walk_goal)
{
    panel_walk_goal_fine_ = panel_walk_goal;
}
void valTask1::setPanelCoeff(const std::vector<float> &panel_coeff)
{
    panel_coeff_ = panel_coeff;
}
