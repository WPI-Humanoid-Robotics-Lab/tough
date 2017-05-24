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
#include <val_controllers/robot_state.h>

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
    chest_controller_    = new chestTrajectory(nh_);
    pelvis_controller_   = new pelvisTrajectory(nh_);
    head_controller_     = new HeadTrajectory(nh_);
    gripper_controller_  = new gripperControl(nh_);
    arm_controller_      = new armTrajectory(nh_);
    wholebody_controller_= new wholebodyManipulation(nh_);

    //state informer
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    map_update_count_ = 0;
    occupancy_grid_sub_ = nh_.subscribe("/map",10, &valTask1::occupancy_grid_cb, this);
    visited_map_sub_    = nh_.subscribe("/visited_map",10, &valTask1::visited_map_cb, this);

    // cartesian planners for the arm
    left_arm_planner_ = new cartesianPlanner("leftPalm", "/world");
    right_arm_planner_ = new cartesianPlanner("rightPalm", "/world");

    // upper body tracker
    upper_body_tracker_ = new upperBodyTracker(nh_);

    // val control common api;s
    control_helper_ = new valControlCommon(nh_);

    // task1 utils
    task1_utils_ = new task1Utils(nh_);

    // grasp state initialised
    prev_grasp_state_ = prevGraspState::NOT_INITIALISED;
}

// destructor
valTask1::~valTask1(){

    // shutdown the subscribers
    visited_map_sub_.shutdown();
    occupancy_grid_sub_.shutdown();

    if(walker_ != nullptr)                delete walker_;
    if(walk_track_ != nullptr)            delete walk_track_;
    if(panel_detector_ != nullptr)        delete panel_detector_;
    if(handle_detector_ != nullptr)       delete handle_detector_;
    if(move_handle_ != nullptr)           delete move_handle_;
    if(chest_controller_ != nullptr)      delete chest_controller_;
    if(pelvis_controller_ != nullptr)     delete pelvis_controller_;
    if(head_controller_ != nullptr)       delete head_controller_;
    if(gripper_controller_ != nullptr)    delete gripper_controller_;
    if(arm_controller_ != nullptr)        delete arm_controller_;
    if(wholebody_controller_ != nullptr)  delete wholebody_controller_;
    if(left_arm_planner_ != nullptr)      delete left_arm_planner_;
    if(right_arm_planner_ != nullptr)     delete right_arm_planner_;
    if(upper_body_tracker_ != nullptr)    delete upper_body_tracker_;
    if(task1_utils_ != nullptr)           delete task1_utils_;
    if(control_helper_ != nullptr)        delete control_helper_;
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

        // reset robot to defaults
        resetRobotToDefaults();

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

        // reset the neck
        head_controller_->moveHead(0.0f, 0.0f, 0.0f, 0.0f);
        ros::Duration(1).sleep();
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

    // set the grasp state
    prev_grasp_state_ = prevGraspState::GRASP_PITCH_HANDLE;

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
        //        chest_controller_->controlChest(0, 10, 40);

        // grasp the handle
        //1 - right
        //3 - left
        // sleep before grasping (to account for sway coming with ches motion)
        ros::Duration(1).sleep();
        geometry_msgs::Pose pose;
        robot_state_->getCurrentPose(VAL_COMMON_NAMES::R_PALM_TF, pose);
        handle_grabber_->grasp_handles(armSide::RIGHT , handle_loc_[PITCH_KNOB_HANDLE]);
        ros::Duration(0.2).sleep(); //wait till grasp is complete
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
    ROS_INFO_STREAM_ONCE("executing " << name);

    static bool execute_once = true;
    static int retry_count = 0;
    static handleDirection rot_dir = handleDirection::ANTICLOCK_WISE;

    //timer
    static std::chrono::system_clock::time_point timer_start = std::chrono::system_clock::now();

    ROS_INFO("time in sec %d ", std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - timer_start).count());

    // execute this part once
    if (execute_once)
    {
        // reset the execute flag
        execute_once = false;

        // set the pelvis height
        // pelvis_controller_->controlPelvisHeight(0.77);

        // generate the way points for the knob (current pose(6DOF) of arm is used to generate the way points)

        //current pose of the hand
        geometry_msgs::Pose current_hand_pose;
        robot_state_->getCurrentPose(VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME, current_hand_pose);

        std::vector<geometry_msgs::Pose> waypoints;
        task1_utils_->getCircle3D(handle_loc_[PITCH_KNOB_CENTER], current_hand_pose.position, current_hand_pose.orientation, panel_coeff_, waypoints, rot_dir, 0.125, 10);
        ROS_INFO("way points generatd");

        ///@todo: remove the visulaisation
        task1_utils_->visulatise6DPoints(waypoints);

        // plan the trajectory
        moveit_msgs::RobotTrajectory traj;
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
        ROS_INFO("trajectory generated");

        // execute the trajectory
        wholebody_controller_->compileMsg(armSide::RIGHT, traj.joint_trajectory);
        ROS_INFO("trajectory sent to controllers");

        // start the timer
        timer_start = std::chrono::system_clock::now();
    }

    // checks to handle the behaviour, which will also trigger necessary transitions

    // if the handle is moving in wrong direction, flip the points
    if(task1_utils_->getValueDirection(task1_utils_->getPitch(), controlSelection::PITCH) == valueDirection::VALUE_INCREASING)
    {
        ROS_INFO("value is incresing");

        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        // set the execute flag
        execute_once = true;

        // flip the direction
        rot_dir = (rot_dir == handleDirection::ANTICLOCK_WISE) ? handleDirection::CLOCK_WISE : handleDirection::ANTICLOCK_WISE;

        // still stay in the same state
        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");
    }
    // if the pitch is corrected
    else if (task1_utils_->isPitchCorrectNow())
    {
        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        ROS_INFO("pitch correct now");

        // wait until the pich correction becomes complete
        ros::Duration(5).sleep();

        //check if its complete
        if (task1_utils_->isPitchCompleted())
        {
            // sucessuflly done
            eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");

            // set the execute flag
            execute_once = true;

            //reset robot to defaults
            resetRobotToDefaults();

            // reset the count
            retry_count = 0;

            ROS_INFO("pitch completed now");
        }
        else
        {
            // still stay in the same state
            eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");
            ROS_INFO("correct but not finished");
        }
    }
    // if we lost the grasp while executing the trajectory
    else if (!robot_state_->isGraspped(armSide::RIGHT))
    {
        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        // redetect the handles (retry)
        eventQueue.riseEvent("/PITCH_CORRECTION_RETRY");

        //set the execute once flag back
        execute_once = true;

        ROS_INFO("grasp lost");
    }
    // if the arm is not moving any more
    else if (task1_utils_->isValueConstant(task1_utils_->getPitch(), controlSelection::PITCH) == valueConstant::VALUE_NOT_CHANGING)
    {
        // replan and execute from the current point,
        // set the execute once state
        execute_once = true;

        // still stay in the same state (as grasp is not lost we will not redetect the handle)
        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");

        ROS_INFO("arm moment stopped, replanning rajectory with the current pose");
    }
    //    // if the arm is not moving any more (ideally its back to the starting point)
    //    ///@todo: not sure if this is required..!!!
    //    /// how do we check if its at the same point..??
    //    else if (!upper_body_tracker_->isArmMoving(armSide::RIGHT, 10))
    //    {
    //        // replan and execute from the current point,
    //        // set the execute once state
    //        execute_once = true;

    //        // still stay in the same state
    //        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");

    //        ROS_INFO("arm moment stopped, replanning rajectory with the current pose");
    //    }
    // if timeout
    // i.e. arm is wobbling around a local minima
    ///@todo use ros timer, instead of system clock
    else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - timer_start).count() > HANDLE_CONTROL_TIMEOUT_SEC) // real time
    {
        ROS_INFO("timeout");

        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        // redetect the handles (retry)
        eventQueue.riseEvent("/PITCH_CORRECTION_RETRY");

        //set the execute once flag back
        execute_once = true;

        // increment the count
        retry_count++;
    }
    // maximum fails
    else if (retry_count > 5)
    {
        //reset robot to default state
        resetRobotToDefaults();

        // reset the count
        retry_count = 0;

        //error state
        eventQueue.riseEvent("/PITCH_CORRECTION_FAILED");
    }
    // fall through case
    else
    {
        // ie. its still executing, stay in the same state
        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");

        ROS_INFO_THROTTLE(2, "executing");
    }

    // generate the event externally (this a fail safe case, should not be required ideally)
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

#if (0)
    static double prev_position_error=9999;
    static bool execute_once = true;
    static int retry_count = 0;
    static double pitch_correction_per_point;
#define PITCH_TEST_ROTATION 10
    double position_error = task1_utils_->getPitchDiff();

    // generate the way points in cartersian space
    std::vector<geometry_msgs::Pose> waypoints;
    //desired pose (i.e. the grab pose)
    geometry_msgs::Pose grab_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::R_END_EFFECTOR_FRAME,grab_pose);


    if (execute_once){
        //do the stuff that needs to be executed only once
        //calculate (change in pitch)/(number of points)
        execute_once = false;
        ROS_INFO_STREAM("Finding pitch correction per point");

        double error_before =  position_error;

        task1_utils_->getCircle3D(handle_loc_[0], handle_loc_[1], grab_pose, panel_coeff_, waypoints, 0.125, 10);

        ///@todo: remove the visulaisation
        task1_utils_->visulatise6DPoints(waypoints);

        // plan the trajectory
        moveit_msgs::RobotTrajectory traj;
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
        traj.joint_trajectory.points.resize(PITCH_TEST_ROTATION);
        ROS_INFO("trajectory points trimmed");

        // execute the trajectory
        wholebody_controller_->compileMsg(armSide::RIGHT, traj.joint_trajectory);
        //        arm_controller_->moveArmTrajectory(armSide::RIGHT, traj.joint_trajectory);
        ros::Duration(4).sleep();

        //stop all trajectories
        double error_after = task1_utils_->getPitchDiff();
        /// @todo what happens when points are not generated correctly?

        pitch_correction_per_point = (error_before - error_after) / PITCH_TEST_ROTATION;
        ROS_INFO_STREAM("Pitch correction per point"<<pitch_correction_per_point);

        prev_position_error = error_after;

        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");
    }

    if(task1_utils_->isPitchCompleted()){
        //completed.
        ROS_INFO_STREAM("Pitch correction checkpoint completed");
        eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");
        //safely release the handle and move arm to a safe position
    }
    else if (task1_utils_->isPitchCorrectNow()){
        // sleep for 3 sec and come back to this state
        // stop all trajectories
        ROS_INFO_STREAM("Pitch is correct. holding on to that position");
        ros::Duration(3).sleep();
        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");
    }
    else if(fabs(position_error) < fabs(prev_position_error) ){
        ROS_INFO_STREAM("Adjusting pitch");
        prev_position_error = position_error;

        int points_to_rotate = position_error*pitch_correction_per_point;
        // generate these many points and rotate handle

        task1_utils_->getCircle3D(handle_loc_[0], handle_loc_[1], grab_pose, panel_coeff_, waypoints, 0.125, 10);

        ///@todo: remove the visulaisation
        task1_utils_->visulatise6DPoints(waypoints);

        // plan the trajectory
        moveit_msgs::RobotTrajectory traj;
        right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
        traj.joint_trajectory.points.resize(points_to_rotate);

        // execute the trajectory
        wholebody_controller_->compileMsg(armSide::RIGHT, traj.joint_trajectory);
        //    arm_controller_->moveArmTrajectory(armSide::RIGHT, traj.joint_trajectory);

        ros::Duration(5).sleep();

        //stop all trajectories
        eventQueue.riseEvent("/PITCH_CORRECTION_EXECUTING");
    }
    else if(fabs(position_error) >= fabs(prev_position_error) && retry_count++ < 5) {
        //failed. change direction and retry
        //want to reset execute_once?
        ROS_INFO_STREAM("current error is more than previous error. retrying");
        execute_once = true;
        prev_position_error=9999;
        eventQueue.riseEvent("/PITCH_CORRECTION_RETRY");
    }
    else{
        //failed completely
        ROS_INFO_STREAM("Pitch correction failed");
        execute_once = true;
        prev_position_error=9999;
        eventQueue.riseEvent("/PITCH_CORRECTION_FAILED");
    }
#endif

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::graspYawHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("executing " << name);

    // set the grasp state
    prev_grasp_state_ = prevGraspState::GRASP_YAW_HANDLE;

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

        // grasp the handle
        //1 - right
        //3 - left
        // sleep before grasping (to account for sway coming with ches motion)
        ros::Duration(1).sleep();
        geometry_msgs::Pose pose;
        robot_state_->getCurrentPose(VAL_COMMON_NAMES::L_PALM_TF, pose);
        handle_grabber_->grasp_handles(armSide::LEFT , handle_loc_[YAW_KNOB_HANDLE]);
        ros::Duration(0.2).sleep(); //wait till grasp is complete
        eventQueue.riseEvent("/GRASP_YAW_HANDLE_EXECUTING");
    }
    else if (robot_state_->isGraspped(armSide::LEFT)){
        ROS_INFO("Grasp is successful");
        eventQueue.riseEvent("/GRASPED_YAW_HANDLE");
    }
    else if (retry_count < 5 ){
        ROS_INFO("Grasp Failed, retrying");
        executing = false;
        ++retry_count;
        eventQueue.riseEvent("/GRASP_YAW_HANDLE_RETRY");
    }
    else {
        ROS_INFO("Failed all conditions. state error");
        eventQueue.riseEvent("/GRASP_YAW_HANDLE_FAILED");
    }

    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask1::controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM_ONCE("executing " << name);

    static bool execute_once = true;
    static int retry_count = 0;
    static handleDirection rot_dir = handleDirection::CLOCK_WISE;

    //timer
    static std::chrono::system_clock::time_point timer_start = std::chrono::system_clock::now();

    // execute this part once
    if (execute_once)
    {
        // reset the execute flag
        execute_once = false;

        // set the pelvis height
        // pelvis_controller_->controlPelvisHeight(0.77);

        // generate the way points for the knob (current pose(6DOF) of arm is used to generate the way points)
        //current pose of the hand
        geometry_msgs::Pose current_hand_pose;
        robot_state_->getCurrentPose(VAL_COMMON_NAMES::L_END_EFFECTOR_FRAME, current_hand_pose);

        std::vector<geometry_msgs::Pose> waypoints;
        task1_utils_->getCircle3D(handle_loc_[YAW_KNOB_CENTER], current_hand_pose.position, current_hand_pose.orientation, panel_coeff_, waypoints, rot_dir, 0.125, 15);
        ROS_INFO("way points generatd");

        ///@todo: remove the visulaisation
        task1_utils_->visulatise6DPoints(waypoints);

        // plan the trajectory
        moveit_msgs::RobotTrajectory traj;
        left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false);
        ROS_INFO("trajectory generated");

        // execute the trajectory
        wholebody_controller_->compileMsg(armSide::LEFT, traj.joint_trajectory);
        ROS_INFO("trajectory sent to controllers");

    }

    // checks to handle the behaviour, which will also trigger necessary transitions
    if(task1_utils_->getValueDirection(task1_utils_->getYaw(), controlSelection::YAW) == valueDirection::VALUE_INCREASING)
    {
        ROS_INFO("value is incresing");

        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        // set the execute flag
        execute_once = true;

        // flip the direction
        rot_dir = (rot_dir == handleDirection::ANTICLOCK_WISE) ? handleDirection::CLOCK_WISE : handleDirection::ANTICLOCK_WISE;

        // still stay in the same state
        eventQueue.riseEvent("/YAW_CORRECTION_EXECUTING");
    }
    // if the yaw is corrected
    else if (task1_utils_->isYawCorrectNow())
    {
        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        ROS_INFO("yaw correct now");

        // wait until the pich correction becomes complete
        ros::Duration(5).sleep();

        //check if its complete
        if (task1_utils_->isYawCompleted())
        {
            // sucessuflly done
            eventQueue.riseEvent("/YAW_CORRECTION_SUCESSFUL");

            // set the execute flag
            execute_once = true;

            //reset robot to defaults
            resetRobotToDefaults();

            // reset the count
            retry_count = 0;

            ROS_INFO("yaw completed now");
        }
        else
        {
            // still stay in the same state
            eventQueue.riseEvent("/YAW_CORRECTION_EXECUTING");
            ROS_INFO("correct but not finished");
        }
    }
    // if we lost the grasp while executing the trajectory
    else if (!robot_state_->isGraspped(armSide::LEFT))
    {
        // redetect the handles (retry)
        eventQueue.riseEvent("/YAW_CORRECTION_RETRY");

        //set the execute once flag back
        execute_once = true;

        ROS_INFO("grasp lost");
    }
    // if the arm is not moving any more
    else if (task1_utils_->isValueConstant(task1_utils_->getYaw(), controlSelection::YAW) == valueConstant::VALUE_NOT_CHANGING)
    {
        // replan and execute from the current point,
        // set the execute once state
        execute_once = true;

        // still stay in the same state (as grasp is not lost we will not redetect the handle)
        eventQueue.riseEvent("/YAW_CORRECTION_EXECUTING");

        ROS_INFO("arm moment stopped, replanning rajectory with the current pose");
    }
    //    // if the arm is not moving any more (ideally its back to the starting point)
    //    ///@todo: not sure if this is required..!!!
    //    /// how do we check if its at the same point..??
    //    else if (!upper_body_tracker_->isArmMoving(armSide::LEFT, 10))
    //    {
    //        // replan and execute from the current point,
    //        // set the execute once state
    //        execute_once = true;

    //        // still stay in the same state
    //        eventQueue.riseEvent("/YAW_CORRECTION_EXECUTING");

    //        ROS_INFO("arm moment stopped, replanning rajectory with the current pose");
    //    }
    // if timeout
    // i.e. arm is wobbling around a local minima
    ///@todo use ros timer, instead of system clock
    else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - timer_start).count() > HANDLE_CONTROL_TIMEOUT_SEC) // real time
    {
        ROS_INFO("timeout");

        // stop all the trajectories
        control_helper_->stopAllTrajectories();

        // redetect the handles (retry)
        eventQueue.riseEvent("/YAW_CORRECTION_RETRY");

        //set the execute once flag back
        execute_once = true;

        // increment the count
        retry_count++;
    }
    // maximum fails
    else if (retry_count > 5)
    {
        //reset robot to default state
        resetRobotToDefaults();

        // reset the count
        retry_count = 0;

        //error state
        eventQueue.riseEvent("/YAW_CORRECTION_FAILED");
    }
    // fall through case
    else
    {
        // ie. its still executing, stay in the same state
        eventQueue.riseEvent("/YAW_CORRECTION_EXECUTING");

        ROS_INFO_THROTTLE(2, "executing");
    }

    /// This state canot go to error state direclty, the transition to error will happen
    /// from the rdetect handles state

    // generate the event externally (this a fail safe case, should not be required ideally)
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::redetectHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    static std::string next_state_transition;
    static int retry_count = 0;
    static bool execute_once = true;

    // reset the robot to defaults (with 0 arm pose)
    resetRobotToDefaults(0);

    if( execute_once)
    {
        // based on the previous state set the state transition
        if (prev_grasp_state_ == prevGraspState::GRASP_PITCH_HANDLE)
        {
            // set the transition
            next_state_transition = "/REDETECTED_HANDLE_GRASP_PITCH";
        }
        else if (prev_grasp_state_ == prevGraspState::GRASP_YAW_HANDLE)
        {
            // set the transition
            next_state_transition = "/REDETECTED_HANDLE_GRASP_YAW";
        }

        // reset the point cloud
        taskCommonUtils::clearPointCloud(nh_);

        // create the object for the pcl detector
        geometry_msgs::Pose pose;
        pose.position.x = panel_walk_goal_fine_.x;
        pose.position.y = panel_walk_goal_fine_.y;
        pose.position.z = 0;
        tf::Quaternion q = tf::createQuaternionFromYaw(panel_walk_goal_fine_.theta);
        tf::quaternionTFToMsg(q, pose.orientation);
        pcl_handle_detector_ = new pcl_handle_detector(nh_, pose);

        // reset the execute once flag
        execute_once = false;
    }

    //get the detections
    std::vector<geometry_msgs::Point> handle_poses;
    // if detection is sucessful
    if (pcl_handle_detector_->getDetections(handle_poses))
    {
        // update the local positions of the handles
        handle_loc_[PITCH_KNOB_HANDLE] = handle_poses[1];
        handle_loc_[YAW_KNOB_HANDLE] = handle_poses[0];

        //set execute once flag
        execute_once = true;

        // reset the state
        prev_grasp_state_ = prevGraspState::NOT_INITIALISED;

        // state transition
        eventQueue.riseEvent(next_state_transition);
    }
    else if (retry_count > 10)
    {
        //reset the count
        retry_count = 0;

        // reset the state
        prev_grasp_state_ = prevGraspState::NOT_INITIALISED;

        // set the execute once flag
        execute_once = true;

        eventQueue.riseEvent("/REDETECT_HANDLE_FAILED");
    }
    else
    {
        // increment the retry count
        retry_count++;
        eventQueue.riseEvent("/REDETECT_HANDLE_RETRY");
    }

    // generate the event externally (this a fail safe case, should not be required ideally)
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

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

    // set the robot to default state to walk
    pelvis_controller_->controlPelvisHeight(0.9);
    ros::Duration(1).sleep();
    chest_controller_->controlChest(0.0, 0.0, 0.0);
    ros::Duration(1).sleep();
    arm_controller_->moveToDefaultPose(armSide::LEFT);
    ros::Duration(1).sleep();
    arm_controller_->moveToDefaultPose(armSide::RIGHT);
    ros::Duration(1).sleep();

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

// helper functions
void valTask1::resetRobotToDefaults(int arm_pose)
{
    // open grippers
    gripper_controller_->openGripper(armSide::RIGHT);
    ros::Duration(1).sleep();
    gripper_controller_->openGripper(armSide::LEFT);
    ros::Duration(1).sleep();

    // increse pelvis
    pelvis_controller_->controlPelvisHeight(0.9);
    ros::Duration(1.0f).sleep();

    // reset chest
    chest_controller_->controlChest(0.0, 0.0, 0.0);
    ros::Duration(1).sleep();

    // arms to default
    if (arm_pose == 0)
    {
        arm_controller_->moveToZeroPose(armSide::LEFT);
        ros::Duration(1).sleep();
        arm_controller_->moveToZeroPose(armSide::RIGHT);
        ros::Duration(1).sleep();
    }
    else if (arm_pose == 1)
    {
        arm_controller_->moveToDefaultPose(armSide::LEFT);
        ros::Duration(1).sleep();
        arm_controller_->moveToDefaultPose(armSide::RIGHT);
        ros::Duration(1).sleep();
    }

    // neck to defaults
    head_controller_->moveHead(0.0f, 0.0f, 0.0f, 0.0f);
}
