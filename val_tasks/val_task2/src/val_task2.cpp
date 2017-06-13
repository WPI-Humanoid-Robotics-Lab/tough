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
#include <val_task2/val_task2.h>
#include <srcsim/StartTask.h>
#include "val_task_common/val_task_common_utils.h"
#include "val_task2/val_task2_utils.h"
#include <queue>
#include <string>

using namespace std;

#define foreach BOOST_FOREACH

valTask2 *valTask2::currentObject = nullptr;

valTask2* valTask2::getValTask2(ros::NodeHandle nh){
    if(currentObject == nullptr){
        currentObject = new valTask2(nh);
        return currentObject;
    }

    ROS_ERROR("valTask2::getValTask2 : Object already exists");
    assert(false && "valTask2::getValTask2 : Object already exists");
}


// constructor and destrcutor
valTask2::valTask2(ros::NodeHandle nh):
    nh_(nh)
{

    // object for the valkyrie walker
    walker_             = new ValkyrieWalker(nh_, 0.7, 0.7, 0, 0.18);
    pelvis_controller_  = new pelvisTrajectory(nh_);
    walk_track_         = new walkTracking(nh_);
    chest_controller_   = new chestTrajectory(nh_);
    arm_controller_     = new armTrajectory(nh_);
    head_controller_    = new HeadTrajectory(nh_);
    gripper_controller_ = new gripperControl(nh_);

    //initialize all detection pointers
    rover_detector_             = nullptr;
    rover_detector_fine_        = nullptr;
    solar_panel_detector_       = nullptr;
    solar_array_detector_       = nullptr;
    solar_array_fine_detector_  = nullptr;
    rover_in_map_blocker_       = nullptr;
    panel_grabber_              = nullptr;
    button_press_               = nullptr;
    cable_task_                 = nullptr;
    finish_box_detector_        = nullptr;

    // use a single multisense object for all stereo detectors. It doesn't work otherwise :(
    ms_sensor_ = new src_perception::MultisenseImage(nh_);
    button_detector_            = new ButtonDetector(nh_, ms_sensor_);
    cable_detector_             = new CableDetector(nh_, ms_sensor_);
    socket_detector_            = new SocketDetector(nh_, ms_sensor_);

    //utils
    task2_utils_    = new task2Utils(nh_);
    task2_utils_->taskLogPub("Starting task 2");
    robot_state_    = RobotStateInformer::getRobotStateInformer(nh_);

    control_common_ = new valControlCommon(nh_);

    // Variables
    map_update_count_ = 0;
    is_rotation_required_ = false;
    panel_grasping_hand_ = armSide::RIGHT;
    // Subscribers
    occupancy_grid_sub_ = nh_.subscribe("/map",10, &valTask2::occupancy_grid_cb, this);
    visited_map_sub_    = nh_.subscribe("/visited_map",10, &valTask2::visited_map_cb, this);
    panel_handle_offset_sub_ = nh_.subscribe("/panel_offset",1, &valTask2::panelHandleOffsetCB, this);
    nudge_sub_ = nh_.subscribe("/nudge_pose",1, &valTask2::nudge_pose_cb, this);

    //publishers
    panel_handle_offset_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    task2_utils_->taskLogPub("Setting Multisense Subscribers");
    cv::Mat img;
    ms_sensor_->giveImage(img);
    ms_sensor_->giveDisparityImage(img);

    // setting skip variables
    skip_3=false;
    skip_4=false;
    skip_6=false;
}

// destructor
valTask2::~valTask2(){
    // shutdown the subscribers
    occupancy_grid_sub_.shutdown();
    visited_map_sub_.shutdown();

    // dereference the pointers
    if (walker_ != nullptr)                    delete walker_;
    if (pelvis_controller_ != nullptr)         delete pelvis_controller_;
    if (walk_track_ != nullptr)                delete walk_track_;
    if (chest_controller_ != nullptr)          delete chest_controller_;
    if (arm_controller_ != nullptr)            delete arm_controller_;
    if (head_controller_ != nullptr)           delete head_controller_;
    if (gripper_controller_ != nullptr)        delete gripper_controller_;
    if (rover_detector_ != nullptr)            delete rover_detector_;
    if (rover_detector_fine_ != nullptr)       delete rover_detector_fine_;
    if (solar_panel_detector_ != nullptr)      delete solar_panel_detector_;
    if (solar_array_detector_ != nullptr)      delete solar_array_detector_;
    if (solar_array_fine_detector_ != nullptr) delete solar_array_fine_detector_;
    if (rover_in_map_blocker_ != nullptr)      delete rover_in_map_blocker_;
    if (panel_grabber_ != nullptr)             delete panel_grabber_;
    if (button_detector_ != nullptr)           delete button_detector_;
    if (button_press_ != nullptr)              delete button_press_;
    if (cable_task_ != nullptr)                delete cable_task_;
    if (cable_detector_ != nullptr)            delete cable_detector_;
    if (socket_detector_ != nullptr)           delete socket_detector_;
    if (finish_box_detector_ != nullptr)       delete finish_box_detector_;
    if (task2_utils_ != nullptr)               delete task2_utils_;
    if (control_common_ != nullptr)            delete control_common_;
}

void valTask2::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){
    // Count the number of times map is updated
    //    ROS_INFO("valTask2::occupancy_grid_cb: map count updated to %d",map_update_count_);
    ++map_update_count_;
}

void valTask2::panelHandleOffsetCB(const std_msgs::Float32 msg)
{
    geometry_msgs::Pose tempPose = solar_panel_handle_pose_;
    float theta = tf::getYaw(tempPose.orientation);
    // converting theta to be along the handle

    theta+=M_PI/2;

    tempPose.position.x = tempPose.position.x + msg.data*cos(theta);
    tempPose.position.y = tempPose.position.y + msg.data*sin(theta);

    setSolarPanelHandlePose(tempPose);

    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time::now();
    marker.ns = "panel_handle";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = solar_panel_handle_pose_;
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    panel_handle_offset_pub_.publish(marker);
}

void valTask2::setPanelRotationFlagCB(const std_msgs::Bool msg)
{
    is_rotation_required_ = msg.data;
}

void valTask2::nudge_pose_cb(const std_msgs::Float64MultiArray msg)
{

    if(msg.data.size()!=5)
    {
        return;
    }

    std::vector<geometry_msgs::Pose> waypoint;
    geometry_msgs::Pose pose;

    armSide side = msg.data[0]== 0 ? armSide::LEFT : armSide::RIGHT;
    if(msg.data[1] ==0) arm_controller_->nudgeArmLocal(side, msg.data[2],msg.data[3],msg.data[4],pose);
    else arm_controller_->nudgeArmPelvis(side, msg.data[2],msg.data[3],msg.data[4],pose);

    waypoint.clear();
    waypoint.push_back(pose);

    task2_utils_->planWholeBodyMotion(side, waypoint);
}

bool valTask2::preemptiveWait(double ms, decision_making::EventQueue& queue) {
    // wait for an external trigger
    for (int i = 0; i < 100 && !queue.isTerminated(); i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));
    return queue.isTerminated();
}
void valTask2::visited_map_cb(const nav_msgs::OccupancyGrid::Ptr msg)
{
    visited_map_ = *msg;
}

// This functions are called based on the remaping in the main.
// when ever a action is published one of these functions will be called
decision_making::TaskResult valTask2::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("valTask2::initTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::initTask : executing " + name);
    static int retry_count = 0;

    // reset mapcount when retrying the state for the first time
    if(retry_count == 0){
        map_update_count_ = 0;
        taskCommonUtils::moveToInitPose(nh_);
    }

    // the state transition can happen from an event externally or can be geenerated here
    ROS_INFO("valTask2::initTask : Occupancy Grid has been updated %d times, tried %d times", map_update_count_, retry_count);
    task2_utils_->taskLogPub("valTask2::initTask : Occupancy Grid has been updated " + std::to_string(map_update_count_)+ "times, tried " + std::to_string(retry_count) + " times");
    if (map_update_count_ > 1) {
        // move to a configuration that is robust while walking
        retry_count = 0;
        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(1.0f).sleep();

        // start the task
//        ros::ServiceClient  client = nh_.serviceClient<srcsim::StartTask>("/srcsim/finals/start_task");
//        srcsim::StartTask   srv;
//        srv.request.checkpoint_id = 1;
//        srv.request.task_id       = 2;
//        if(client.call(srv)) {
//            //what do we do if this call fails or succeeds?
//        }
//        else
//        {
//            ROS_ERROR("valTask2::initTask : service not called");
//            task2_utils_->taskLogPub("valTask2::initTask : service not called");
//            eventQueue.riseEvent("/INIT_FAILED");
//        }
        // generate the event
        head_controller_->moveHead(0,0,0);
        eventQueue.riseEvent("/INIT_SUCESSFUL");

    }
    else if (map_update_count_ < 2 && retry_count++ < 40) {
        // to get a better point cloud at the beginning
        if(retry_count == 1) head_controller_->moveHead(0,0,20);
        if(retry_count == 3) head_controller_->moveHead(0,0,-20);


        ROS_INFO("valTask2::initTask : Wait for occupancy grid to be updated with atleast 2 messages");
        task2_utils_->taskLogPub("valTask2::initTask : Wait for occupancy grid to be updated with atleast 2 messages");
        ros::Duration(2.0).sleep();
        eventQueue.riseEvent("/INIT_RETRY");
    }
    else {
        retry_count = 0;
        ROS_INFO("valTask2::initTask : Failed to initialize");
        task2_utils_->taskLogPub("valTask2::initTask : Failed to initialize");
        eventQueue.riseEvent("/INIT_FAILED");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::deployPanelTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectRoverTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::detectRoverTask : executing " + name);

    if(rover_detector_ == nullptr){
        rover_detector_ = new RoverDetector(nh_);
    }

    static int retry_count = 0;
    static int fail_count = 0;
    std::vector<std::vector<geometry_msgs::Pose> > roverPoseWaypoints;
    rover_detector_->getDetections(roverPoseWaypoints);


    ROS_INFO("valTask2::detectRoverTask : Size of poses : %d", (int)roverPoseWaypoints.size());
    task2_utils_->taskLogPub("valTask2::detectRoverTask : Size of poses :" + std::to_string((int)roverPoseWaypoints.size()));

    //    consider the rover detected if there are more than 1 detections
    if (roverPoseWaypoints.size() > 1 ) {

        // Detections are 3D but we need 2D pose.So convert it here
        std::vector<geometry_msgs::Pose2D> detectedPoses2D;
        int length = roverPoseWaypoints.back().size();
        int Idx = roverPoseWaypoints.size() -1 ;
        for (int i = 0; i < length ; ++i){
            geometry_msgs::Pose2D pose2D;
            // get the last detected pose
            pose2D.x = roverPoseWaypoints[Idx][i].position.x;
            pose2D.y = roverPoseWaypoints[Idx][i].position.y;

            std::cout << "x " << pose2D.x << " y " << pose2D.y << std::endl;

            // get the theta
            pose2D.theta = tf::getYaw(roverPoseWaypoints[Idx][i].orientation);
            detectedPoses2D.push_back(pose2D);
        }
        setRoverWalkGoal(detectedPoses2D);


        ROVER_SIDE roverSide;
        if(rover_detector_->getRoverSide(roverSide)){
            setRoverSide(roverSide == ROVER_SIDE::RIGHT); /// check
            // block rover in /map
            //            rover_in_map_blocker_ = new SolarArrayDetector(nh_, detectedPoses2D.back(),is_rover_on_right_);
            //wait for the map to update. This is required to ensure the footsteps dont collide with rover
            //        ros::Duration(0.5).sleep();
            taskCommonUtils::moveToWalkSafePose(nh_);

            retry_count = 0;
            fail_count = 0;
            // update the plane coeffecients
            eventQueue.riseEvent("/DETECTED_ROVER");
            ROS_INFO("detected rover");
            task2_utils_->taskLogPub("detected rover");
            if(rover_detector_ != nullptr) delete rover_detector_;
            rover_detector_ = nullptr;
        }
        else{
            eventQueue.riseEvent("/DETECT_ROVER_RETRY");
        }
    }
    else if(retry_count < 5) {
        ROS_INFO("sleep for 3 seconds for panel detection, retry count : %d fail count : %d",retry_count,fail_count);
        task2_utils_->taskLogPub("sleep for 3 seconds for panel detection, retry count : " + std::to_string(retry_count) + "fail count :" + std::to_string(fail_count));
        ++retry_count;
        ros::Duration(2).sleep();
        eventQueue.riseEvent("/DETECT_ROVER_RETRY");
    }   else if (fail_count < 5)    {
        // increment the fail count
        fail_count++;
        task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);
        eventQueue.riseEvent("/DETECT_ROVER_RETRY");
    }

    // if failed for more than 5 times, go to error state
    else {
        // reset the fail count
        fail_count = 0;
        retry_count = 0;
        ROS_INFO("Rover detection failed");
        task2_utils_->taskLogPub("Rover detection failed");
        eventQueue.riseEvent("/DETECT_ROVER_FAILED");
        if(rover_detector_ != nullptr) delete rover_detector_;
        rover_detector_ = nullptr;
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
        task2_utils_->taskLogPub("waiting for transition from "+ name);
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM_ONCE("executing " << name);

    // walk to the goal location
    // the goal can be updated on the run time
    static bool executeOnce = true;
    static geometry_msgs::Pose2D goal , pose_prev;

    static int fail_count = 0;
    static std::queue<geometry_msgs::Pose2D> goal_waypoints;
    // Run this block once during every visit of the state from either a failed state or previous state
    if(executeOnce){
        task2_utils_->taskLogPub("executing " + name);
        std::queue<geometry_msgs::Pose2D>  temp1;
        goal_waypoints = temp1;
        for (auto pose : rover_walk_goal_waypoints_){
            ROS_INFO("X: %.2f  Y:%.2f  Theta:%.2f", pose.x, pose.y, pose.theta);
            task2_utils_->taskLogPub("X: " + std::to_string(pose.x) + " Y: " + std::to_string(pose.y) + " Theta: " + std::to_string(pose.theta));
            goal_waypoints.push(pose);
        }
        goal = goal_waypoints.front();
        goal_waypoints.pop();
        fail_count = 0;
        executeOnce = false;
        geometry_msgs::Pose2D  temp;
        pose_prev = temp;
    }

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);
    // Robot will walk to coarse goal first. once that is reached, goal is changed to fine goal and the flag variable is updated
    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, goal) ) {
        ROS_INFO("Local goal reached. Number of waypoints remaining %d", goal_waypoints.size());
        task2_utils_->taskLogPub("Local goal reached. Number of waypoints remaining " +  std::to_string( goal_waypoints.size()));
        // if coarse goal was already reached before, then the current state is fine goal reached, else it is coarse goal reached
        if(goal_waypoints.empty()){
            ROS_INFO("reached The rover");
            task2_utils_->taskLogPub("reached the rover");
            ros::Duration(3).sleep(); // This is required for steps to complete
            eventQueue.riseEvent("/REACHED_ROVER");
            executeOnce = true;
            ros::Duration(1).sleep();
        }
        else
        {
            ROS_INFO("%d more waypoints to go", goal_waypoints.size());
            task2_utils_->taskLogPub(std::to_string( goal_waypoints.size()) + "more waypoints to go");
            goal= goal_waypoints.front();
            goal_waypoints.pop();
            ros::Duration(3).sleep();
            eventQueue.riseEvent("/WALK_EXECUTING");
        }
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, goal)) {
        ROS_INFO_STREAM("pose chaned to "<<goal);
        task2_utils_->taskLogPub("pose chaned to  x :" + std::to_string(goal.x) + "y: " +std::to_string(goal.y) + "theta : " + std::to_string(goal.theta) );
        walker_->walkToGoal(goal, false);
        // sleep so that the walk starts
        ROS_INFO("Footsteps should be generated now");
        task2_utils_->taskLogPub("Footsteps should be generated now");
        ros::Duration(4).sleep();
        // update the previous pose
        pose_prev = goal;
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
        // reset all the static variables in next run
        executeOnce = true;
        ROS_INFO("walk failed");
        task2_utils_->taskLogPub("walk failed");
        eventQueue.riseEvent("/WALK_FAILED");
    }
    // if failed retry detecting the panel and then walk
    else
    {
        // increment the fail count
        fail_count++;
        executeOnce = true;
        ROS_INFO("walk retry");
        task2_utils_->taskLogPub("walk failed");
        eventQueue.riseEvent("/WALK_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
        task2_utils_->taskLogPub("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectPanelTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::detectPanelTask : executing " + name);
    static bool isFirstRun = true;

    if(isFirstRun){
        head_controller_->moveHead(0,40,0);
        ros::Duration(2).sleep();
    }
    if(solar_panel_detector_ == nullptr) {
        if (!isFirstRun){
            ROS_INFO("valTask2::detectPanelTask : Clearing pointcloud");
            task2_utils_->taskLogPub("valTask2::detectPanelTask : Clearing pointcloud");
            task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::LARGE_BOX);
        }

        task2_utils_->resumePointCloud();
        isFirstRun = false;
        geometry_msgs::Point temp;
        button_coordinates_temp_ = temp;
        int retry = 0;
        while (!button_detector_->findButtons(button_coordinates_temp_) && retry++ < 10);
        ROS_INFO("valTask2::detectPanelTask : Button detected at x:%f y:%f z:%f", button_coordinates_temp_.x,button_coordinates_temp_.y,button_coordinates_temp_.z);

        solar_panel_detector_ = new SolarPanelDetect(nh_, button_coordinates_temp_);
        chest_controller_->controlChest(2, 2, 2);

        //move arms to default position
        arm_controller_->moveToDefaultPose(armSide::RIGHT);
        gripper_controller_->openGripper(armSide::RIGHT);
        ros::Duration(0.2).sleep();

        arm_controller_->moveToDefaultPose(armSide::LEFT);
        gripper_controller_->openGripper(armSide::LEFT);
        ros::Duration(0.2).sleep();
    }

    static int retry_count = 0;

    // detect solar panel
    std::vector<geometry_msgs::Pose> poses;
    ROS_INFO("valTask2::detectPanelTask : get detections");
    task2_utils_->taskLogPub("valTask2::detectPanelTask : get detections");
    solar_panel_detector_->getDetections(poses);
    ROS_INFO("valTask2::detectPanelTask : Size of detections : %d", poses.size());
    task2_utils_->taskLogPub("valTask2::detectPanelTask : Size of detections : " + std::to_string(poses.size()));
    // if we get atleast two detections
    if (poses.size() > 1)
    {
        size_t idx = poses.size()-1;
        setSolarPanelHandlePose(poses[idx]);
        task2_utils_->reOrientTowardsGoal(solar_panel_handle_pose_.position);
        ros::Duration(2.0).sleep();

        task2_utils_->taskLogPub("valTask2::detectPanelTask : Button detected at x: " + std::to_string(button_coordinates_temp_.x) + " y: " + std::to_string(button_coordinates_temp_.y) + " z:" + std::to_string(button_coordinates_temp_.z));
        ROS_INFO_STREAM("valTask2::detectPanelTask : Position " << poses[idx].position.x<< " " <<poses[idx].position.y <<" "<<poses[idx].position.z);
        task2_utils_->taskLogPub("valTask2::detectPanelTask : Position " + std::to_string(poses[idx].position.x) + " "  + std::to_string(poses[idx].position.y) + " " + std::to_string(poses[idx].position.z ));
        ROS_INFO_STREAM("valTask2::detectPanelTask : quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w);
        task2_utils_->taskLogPub("valTask2::detectPanelTask : quat " + std::to_string(poses[idx].orientation.x) + " "  + std::to_string(poses[idx].orientation.y) + " " + std::to_string(poses[idx].orientation.z ) + " " + std::to_string(poses[idx].orientation.w));
        retry_count = 0;

        eventQueue.riseEvent("/DETECTED_PANEL");
//        eventQueue.riseEvent("/MANUAL_EXECUTION");
        if(solar_panel_detector_ != nullptr) delete solar_panel_detector_;
        solar_panel_detector_ = nullptr;
    }

    else if(retry_count < 10) {
        ROS_INFO("valTask2::detectPanelTask :sleep for 3 seconds for panel detection");
        task2_utils_->taskLogPub("valTask2::detectPanelTask :sleep for 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_PANEL_RETRY");
        ros::Duration(3).sleep();
    }
    // if failed for more than 10 times, go to error state
    else
    {
        retry_count = 0;
        isFirstRun = true;
        eventQueue.riseEvent("/DETECT_PANEL_FAILED");
        if(solar_panel_detector_ != nullptr) delete solar_panel_detector_;
        solar_panel_detector_ = nullptr;

        //        if(button_detector_ != nullptr) delete button_detector_;
        //        button_detector_ = nullptr;
    }


    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectPanelTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::graspPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::graspPanelTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::graspPanelTask : executing " + name);
    // we reached here means we don't need the map blocker anymore.
    if (rover_in_map_blocker_ != nullptr){
        delete rover_in_map_blocker_;
        rover_in_map_blocker_  = nullptr;
    }

    if(panel_grabber_ == nullptr){
        panel_grabber_ = new solar_panel_handle_grabber(nh_);
    }

    /*  Find which hand is required for grasping
     *  Find if rotation is required
     *  Plan trajectory and execute
     *  If success == go to pick panel state
     *  If fail == redetect panel
     *  retry for 15 times
     */

    static int retry_count = 0;
    static armSide hand;

    if(retry_count == 10){
        // reducing the pelvis height for the next 10 trials
        pelvis_controller_->controlPelvisHeight(0.8);
    }

    if(retry_count< 20){
        setSolarPanelHandlePose(task2_utils_->grasping_hand(hand,solar_panel_handle_pose_));
        setPanelGraspingHand(hand);
        is_rotation_required_=task2_utils_->isRotationReq(hand,solar_panel_handle_pose_.position,button_coordinates_temp_);

        ROS_INFO("valTask2::graspPanelTask : Rotation is %d ", is_rotation_required_);
        task2_utils_->taskLogPub("valTask2::graspPanelTask : Rotation is  " + std::to_string(is_rotation_required_));
        //Pause the pointcloud to avoid obstacles on map due to panel
        task2_utils_->pausePointCloud();
        retry_count++;

        if (panel_grabber_->grasp_handles(hand, solar_panel_handle_pose_, is_rotation_required_)) {
            ROS_INFO("valTask2::graspPanelTask : Plan is 100 % Maybe Grasp is successful. Going to Pick Pannel Task");
            task2_utils_->taskLogPub("valTask2::graspPanelTask : Plan is 100 % Maybe Grasp is successful. Going to Pick Pannel Task");
            eventQueue.riseEvent("/GRASPED_PANEL");
            task2_utils_->taskLogPub("valTask2::graspPanelTask : Moving panel closer to chest to avoid collision with trailer");
            task2_utils_->afterPanelGraspPose(panel_grasping_hand_, is_rotation_required_);
        }
        else
        {
            ROS_INFO("valTask2::graspPanelTask :100 %. retry count = %d",retry_count);
            task2_utils_->taskLogPub("valTask2::graspPanelTask :All conditions failed. retry count =  " + std::to_string(retry_count));
            eventQueue.riseEvent("/REDETECT_PANEL");
        }
    }
    else
    {
        ROS_INFO("valTask2::graspPanelTask :All conditions failed. retry count = %d",retry_count);
        task2_utils_->taskLogPub("valTask2::graspPanelTask :All conditions failed. retry count = " + std::to_string(retry_count));
        pelvis_controller_->controlPelvisHeight(0.9);
        eventQueue.riseEvent("/GRASP_PANEL_FAILED");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::graspPanelTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::graspPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::pickPanelTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::pickPanelTask : executing " + name);
    if(task2_utils_->afterPanelGraspPose(panel_grasping_hand_, is_rotation_required_)){

        ROS_INFO("valTask2::pickPanelTask : Walking 1 step back");
        task2_utils_->taskLogPub("valTask2::pickPanelTask : Walking 1 step back");
        // increasing pelvis height back to normal
        pelvis_controller_->controlPelvisHeight(0.9);
        task2_utils_->movePanelToWalkSafePose(panel_grasping_hand_, is_rotation_required_);
        ros::Duration(1).sleep();
        std::vector<float> x_offset={-0.3,-0.3};
        std::vector<float> y_offset={0.0,0.1};
        walker_->walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);
        ros::Duration(4).sleep();

        // this is absolutely necessary
        task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);

        // walk slowly for this turn
        walker_->setWalkParms(1.0, 1.0, 0);
        /// @todo The following code should be a new state
        // reorient the robot.

        ROS_INFO("valTask2::pickPanelTask : Reorienting");
        task2_utils_->taskLogPub("valTask2::pickPanelTask : Reorienting");
        geometry_msgs::Pose pose;
        pose.position.x = 0.0;
        pose.position.y= is_rover_on_right_ == true ? 0.5 : -0.5;
        pose.orientation.w = 1.0;
        robot_state_->transformPose(pose, pose, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
        geometry_msgs::Pose2D pose2D;
        pose2D.x = pose.position.x;
        pose2D.y = pose.position.y;
        pose2D.theta = rover_walk_goal_waypoints_.front().theta;
        ROS_INFO("valTask2::pickPanelTask: Walking to x:%f y:%f theta:%f", pose2D.x,pose2D.y,pose2D.theta);
        task2_utils_->taskLogPub("valTask2::pickPanelTask: Walking to x: " + std::to_string(pose2D.x) + " y: " + std::to_string(pose2D.y) + " theta: " + std::to_string(pose2D.theta));
        walker_->walkToGoal(pose2D);
        ros::Duration(1).sleep();
        walker_->setWalkParms(0.7, 0.7, 0);
        head_controller_->moveHead(0,0,0);
        ros::Duration(2).sleep();
//        eventQueue.riseEvent("/PICKED_PANEL");
        // lets detect the array manually
        eventQueue.riseEvent("/MANUAL_EXECUTION");
    }
    else
    {
        eventQueue.riseEvent("/RE_REDETECT_PANEL");
    }


    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::pickPanelTask waiting for transition");
        task2_utils_->taskLogPub("valTask2::pickPanelTask waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectSolarArrayTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : executing " + name);

    if(solar_array_detector_ == nullptr) {
        // this is absolutely necessary
        task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);

        //this blocks rover on map
        solar_array_detector_ = new SolarArrayDetector(nh_, rover_walk_goal_waypoints_.back(), is_rover_on_right_);
        ros::Duration(0.2).sleep();
    }

    //    ///@todo check if the threshold is right for this function. since the hand is supporting the button, condition might fail
    //    if (task2_utils_->isPanelPicked(panel_grasping_hand_)){
    //        ROS_INFO("Panel is still in hand");
    //    }
    //    else
    //    {
    //        ROS_INFO("Dropped the bag on the way. Consider skipping checkpoint");
    //        eventQueue.riseEvent("/DETECT_ARRAY_FAILED");
    //    }

    static int fail_count = 0;
    static int retry_count = 0;

    static bool executeOnce =true;

    static std::queue<float> head_yaw_ranges;
    if(executeOnce)
    {
        std::queue<float> temp;
        head_yaw_ranges= temp;
        head_yaw_ranges.push(-35.0);
        head_yaw_ranges.push(-15.0);
        head_yaw_ranges.push( 15.0);
        head_yaw_ranges.push( 35.0);
        executeOnce=false;
    }

    // detect solar array
    std::vector<geometry_msgs::Pose> poses;
    solar_array_detector_->getDetections(poses);

    // if we get atleast two detections
    if (poses.size() > 1) {
        // get the last detected pose
        int idx = poses.size() -1 ;

        geometry_msgs::Pose2D pose2D;
        pose2D.x = poses[idx].position.x;
        pose2D.y = poses[idx].position.y;
        // get the theta
        pose2D.theta = tf::getYaw(poses[idx].orientation);

        setSolarArrayWalkGoal(pose2D);

        ROS_INFO("valTask2::detectSolarArrayTask : Position x:%f y:%f z:%f",poses[idx].position.x,poses[idx].position.y,poses[idx].position.z);
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : Position x: " + std::to_string(poses[idx].position.x) + " y: " + std::to_string(poses[idx].position.y) + " z: " + std::to_string(poses[idx].position.z));
        ROS_INFO("valTask2::detectSolarArrayTask : yaw:%f",pose2D.theta);
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : yaw:" + std::to_string(pose2D.theta));
        retry_count = 0;

        if(solar_array_detector_ != nullptr) delete solar_array_detector_;
        solar_array_detector_ = nullptr;
        head_controller_->moveHead(0,0,0);
        eventQueue.riseEvent("/DETECTED_ARRAY");

    }

    else if(retry_count < 3) {
        ROS_INFO("valTask2::detectSolarArrayTask : sleep 3 seconds for panel detection");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : sleep 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_ARRAY_RETRY");
        ros::Duration(3).sleep();
    }
    // if failed for more than 5 times, go to error state
    else if (fail_count > 4)
    {
        // reset the fail count
        ROS_INFO("valTask2::detectSolarArrayTask : Failed 5 times. transitioning to error state");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : Failed 5 times. transitioning to error state");
        retry_count=0;
        fail_count = 0;
        eventQueue.riseEvent("/DETECT_ARRAY_FAILED");
        executeOnce=true;
        head_controller_->moveHead(0,0,0,2.0f);
        if(solar_array_detector_ != nullptr) delete solar_array_detector_;
        solar_array_detector_ = nullptr;
    }
    // if failed retry detecting the panel
    else
    {
        ROS_INFO("valTask2::detectSolarArrayTask : Failed attempt. trying again");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : Failed attempt. trying again");
        // increment the fail count
        if(!head_yaw_ranges.empty())
        {
            head_controller_->moveHead(0,0,head_yaw_ranges.front());
            head_yaw_ranges.pop();
        }
        retry_count=0;
        fail_count++;
        eventQueue.riseEvent("/DETECT_ARRAY_RETRY");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM_ONCE("valTask2::walkSolarArrayTask : executing " << name);

    static bool executeOnce = true;
    if(executeOnce )
    {
        task2_utils_->clearCurrentPoseMap();
        executeOnce = false;
    }
    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    /// is this required?
    //task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);

    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, solar_array_walk_goal_) ) {
        ROS_INFO("valTask2::walkSolarArrayTask : reached solar array");
        task2_utils_->taskLogPub("valTask2::walkSolarArrayTask : reached solar array");
        ros::Duration(1).sleep();
        /// @todo: check if robot rechead the panel (is this done ???)
        geometry_msgs::Pose2D temp; // added to test
        pose_prev = temp; // added to test
        eventQueue.riseEvent("/REACHED_ARRAY");
        executeOnce = true;
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, solar_array_walk_goal_)) {
        ROS_INFO_STREAM("valTask2::walkSolarArrayTask : pose chaned to "<<solar_array_walk_goal_);
        task2_utils_->taskLogPub("valTask2::walkSolarArrayTask : pose chaned to x: " + std::to_string(solar_array_walk_goal_.x) + "y: " + std::to_string(solar_array_walk_goal_.y) + "theta: " + std::to_string(solar_array_walk_goal_.theta));
        walker_->walkToGoal(solar_array_walk_goal_, false);
        // sleep so that the walk starts
        ROS_INFO("valTask2::walkSolarArrayTask : Footsteps should be generated now");
        task2_utils_->taskLogPub("valTask2::walkSolarArrayTask : Footsteps should be generated now");
        ros::Duration(4).sleep();
        // update the previous pose
        pose_prev = solar_array_walk_goal_;
        eventQueue.riseEvent("/WALK_TO_ARRAY_EXECUTING");
    }
    // if walking stay in the same state
    else if (walk_track_->isWalking())
    {
        // no state change
        task2_utils_->resumePointCloud();
        ROS_INFO_THROTTLE(2, "valTask2::walkSolarArrayTask : walking");
        eventQueue.riseEvent("/WALK_TO_ARRAY_EXECUTING");
    }
    // if walk finished
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        geometry_msgs::Pose2D temp;
        pose_prev = temp;
        task2_utils_->resumePointCloud();
        ROS_INFO("valTask2::walkSolarArrayTask : walk failed");
        task2_utils_->taskLogPub("valTask2::walkSolarArrayTask : walk failed");
        eventQueue.riseEvent("/WALK_TO_ARRAY_FAILED");
        executeOnce = true;
    }
    // if failed retry detecting the array and then walk
    else
    {
        // increment the fail count
        fail_count++;
        geometry_msgs::Pose2D temp;
        pose_prev = temp;
        ROS_INFO("valTask2::walkSolarArrayTask : walk retry");
        task2_utils_->taskLogPub("valTask2::walkSolarArrayTask : walk retry");
        eventQueue.riseEvent("/WALK_TO_ARRAY_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::walkSolarArrayTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::walkSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::rotatePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::rotatePanelTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::rotatePanelTask : executing " + name);
    if (!is_rotation_required_)
    {
        ROS_INFO("valTask2::rotatePanelTask : Rotation is not required. skipping this state");
        task2_utils_->taskLogPub("valTask2::rotatePanelTask : Rotation is not required. skipping this state");
    }
    else
    {
        ROS_INFO("valTask2::rotatePanelTask : Rotating the panel");
        task2_utils_->taskLogPub("valTask2::rotatePanelTask : Rotating the panel");
        task2_utils_->rotatePanel(panel_grasping_hand_);
    }
    eventQueue.riseEvent("/ROTATED_PANEL");

    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::rotatePanelTask waiting for transition");
        task2_utils_->taskLogPub("valTask2::rotatePanelTask waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::findCableIntermediateTask(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
    ROS_INFO_STREAM("valTask2::findCableIntermediateTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::findCableIntermediateTask : executing " + name);
    static std::queue<float> head_yaw_ranges;

    static bool executingOnce = true;
    static float q1, q3;
    static bool isPoseChangeReq = true;

    if(skip_3)
    {
        ROS_INFO("valTask2::findCableIntermediateTask : [SKIP] Disabled pose change required flag");
        task2_utils_->taskLogPub("valTask2::findCableIntermediateTask : [SKIP] Disabled pose change required flag");
        isPoseChangeReq = false;
    }

    if(executingOnce)
    {
        if(isPoseChangeReq)
        {
            // moving hands to get in position to detect the cable
            ROS_INFO("valTask2::findCableIntermediateTask : Moving panel to see cable");
            task2_utils_->taskLogPub("valTask2::findCableIntermediateTask : Moving panel to see cable");
            q3 = panel_grasping_hand_ == armSide::LEFT ? -0.5 : 0.5;
            arm_controller_->moveArmJoint((armSide)!panel_grasping_hand_, 3, q3);
            ros::Duration(0.5).sleep();
            q1 = panel_grasping_hand_ == armSide::LEFT ? -0.36 : 0.36;
            arm_controller_->moveArmJoint(panel_grasping_hand_,1,q1);
            ros::Duration(0.5).sleep();
        }

        head_controller_->moveHead(0,30,0,1.5);
        ros::Duration(2).sleep();

        // possible head yaw rotations to better detect the cable
        //clear the queue before starting
        std::queue<float> temp;
        head_yaw_ranges= temp;
        head_yaw_ranges.push(-55.0);
        head_yaw_ranges.push(-45.0);
        head_yaw_ranges.push(-35.0);
        head_yaw_ranges.push(-20.0);
        head_yaw_ranges.push(-10.0);
        head_yaw_ranges.push( 10.0);
        head_yaw_ranges.push( 20.0);
        head_yaw_ranges.push( 35.0);
        head_yaw_ranges.push( 45.0);
        head_yaw_ranges.push( 55.0);
        executingOnce = false;
        control_common_->stopAllTrajectories();
    }

    geometry_msgs::Point cable_point;
    int retry = 0;
    while (!cable_detector_->findCable(cable_point) && retry++ < 5);
    std::cout<<"valTask2::findCableIntermediateTask : retry count : "<<retry<<"\n";
    std::cout<<"cable_point x: "<<cable_point.x<<" cable_point y: "<<cable_point.y<<" cable_point z: "<<cable_point.z<<"\n";

    if(cable_point.x == 0 && !head_yaw_ranges.empty())
    {
        // wrong point detected. move head and try again
        head_controller_->moveHead(0,30,head_yaw_ranges.front(), 2.0f);
        ROS_INFO("valTask2::findCableIntermediateTask : Cable not found. Trying with %.6f angle",head_yaw_ranges.front());
        task2_utils_->taskLogPub("valTask2::findCableIntermediateTask : Cable not found. Trying with " + std::to_string(head_yaw_ranges.front()) + "angle");
        head_yaw_ranges.pop();
        ros::Duration(2.5).sleep();
        control_common_->stopAllTrajectories();
        eventQueue.riseEvent("/FIND_CABLE_RETRY");
    }
    else if (cable_point.x == 0)
    {
        // state failed after detecting 5*4 times
        ROS_INFO("valTask2::findCableIntermediateTask : Cable not found. Failed after 20 trials");
        task2_utils_->taskLogPub("valTask2::findCableIntermediateTask : Cable not found. Failed after 20 trials");
        eventQueue.riseEvent("/FIND_CABLE_FAILED");
        head_controller_->moveHead(0,0,0,2.0f);
        isPoseChangeReq = true;
        executingOnce= true;
    }
    else
    {
        // cable found
        setTempCablePoint(cable_point);
        std::cout<<"Cable position x: "<<cable_point.x<<"\t"<<"Cable position y: "<<cable_point.y<<"\t"<<"Cable position z: "<<cable_point.z<<"\n";
        ROS_INFO("valTask2::findCableIntermediateTask : Cable found! Setting motion back. exiting");
        task2_utils_->taskLogPub("valTask2::findCableIntermediateTask : Cable found! Setting motion back. exiting");

        eventQueue.riseEvent("/FOUND_CABLE");

        if(isPoseChangeReq)
        {
            // set the poses back
            q3 = panel_grasping_hand_ == armSide::LEFT ? -1.85 : 1.85;
            q1 = panel_grasping_hand_ == armSide::LEFT ? -1.04 : 1.04;
            float q0 = -1.85;
            arm_controller_->moveArmJoint(panel_grasping_hand_,1,q1);
            ros::Duration(0.2).sleep();
            arm_controller_->moveArmJoint((armSide)!panel_grasping_hand_, 3, q3);
            ros::Duration(0.2).sleep();
            arm_controller_->moveArmJoint(panel_grasping_hand_,0,q0);
            ros::Duration(2).sleep();
        }
        head_controller_->moveHead(0,0,0,2.0f);
        executingOnce= true;
        isPoseChangeReq = true;
    }


    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : waiting for transition");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectSolarArrayFineTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectSolarArrayFineTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::detectSolarArrayFineTask : executing " + name);

    static int retry_count = 0;
    if(solar_array_fine_detector_ == nullptr) {
        task2_utils_->clearPointCloud();
        task2_utils_->resumePointCloud();
        solar_array_fine_detector_ = new ArrayTableDetector(nh_, cable_point_temp_);
    }

    // detect solar array
    std::vector<geometry_msgs::Pose> poses;
    solar_array_fine_detector_->getDetections(poses);

    // if we get atleast two detections
    ROS_INFO("valTask2::detectSolarArrayFineTask : size of detections %d ",poses.size());
    if (poses.size() > 1) {
        // get the last detected pose
        int idx = poses.size() -1 ;

        geometry_msgs::Pose2D pose2D;
        pose2D.x = poses[idx].position.x;
        pose2D.y = poses[idx].position.y;
        // get the theta
        pose2D.theta = tf::getYaw(poses[idx].orientation);

        setSolarArrayFineWalkGoal(pose2D);
        ROS_INFO("valTask2::detectSolarArrayFineTask : detected panel");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayFineTask : detected panel");
        ROS_INFO("valTask2::detectSolarArrayFineTask : Position x:%f y:%f z:%f",poses[idx].position.x,poses[idx].position.y,poses[idx].position.z);
        task2_utils_->taskLogPub("valTask2::detectSolarArrayFineTask : Position x: " + std::to_string(poses[idx].position.x) + " y: " + std::to_string(poses[idx].position.y) + " z: " + std::to_string(poses[idx].position.z));
        ROS_INFO("valTask2::detectSolarArrayFineTask : yaw:%f",pose2D.theta);
        retry_count = 0;

        eventQueue.riseEvent("/DETECTED_ARRAY_FINE");

        if(solar_array_fine_detector_ != nullptr) delete solar_array_fine_detector_;
        solar_array_fine_detector_ = nullptr;

        pelvis_controller_->controlPelvisHeight(1.0);
        ros::Duration(2).sleep();
    }

    else if(retry_count < 5) {
        ROS_INFO("valTask2::detectSolarArrayFineTask : sleep 3 seconds for panel detection");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayFineTask : sleep 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_ARRAY_FINE_RETRY");
        ros::Duration(3).sleep();
    }
    // if failed for more than 5 times, go to error state
    else
    {
        ROS_INFO("valTask2::detectSolarArrayFineTask : Failed 5 times. transitioning to error state");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayFineTask : Failed 5 times. transitioning to error state");
        retry_count = 0;
        eventQueue.riseEvent("/DETECT_ARRAY_FINE_FAILED");
        if(solar_array_fine_detector_ != nullptr) delete solar_array_fine_detector_;
        solar_array_fine_detector_ = nullptr;
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::alignSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM_ONCE("valTask2::alignSolarArrayTask : executing " << name);
    
    static int fail_count = 0;
    static bool executeOnce = true;
    if(skip_3 || skip_4)
    {
        ROS_INFO("valTask2::alignSolarArrayTask: Resetting robot");
        control_common_->resetRobot();
        ros::Duration(0.2).sleep();
        task2_utils_->clearCurrentPoseMap();
        skip_3 = false;
        executeOnce = false;
        if(skip_4)
        {
            arm_controller_->moveToZeroPose(armSide::LEFT); /// to account for panel being very close to body
            ros::Duration(1).sleep();
        }
    }

    if(executeOnce)
    {
        task2_utils_->clearCurrentPoseMap();
        executeOnce = false;
    }

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);
    task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);
    ros::Duration(1).sleep();
    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, solar_array_fine_walk_goal_) ) {
        ROS_INFO_STREAM("valTask2::alignSolarArrayTask : pose changed to x: " << solar_array_fine_walk_goal_);
        task2_utils_->taskLogPub("valTask2::alignSolarArrayTask : pose chaned to x: " + std::to_string(solar_array_fine_walk_goal_.x) + "y: " + std::to_string(solar_array_fine_walk_goal_.y) + " theta:" + std::to_string(solar_array_fine_walk_goal_.theta));
        ros::Duration(1).sleep();
        geometry_msgs::Pose2D temp;
        pose_prev = temp;
        // TODO: check if robot rechead the panel
        executeOnce = true;
//        eventQueue.riseEvent("/ALIGNED_TO_ARRAY");
        eventQueue.riseEvent("/MANUAL_EXECUTION");
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, solar_array_fine_walk_goal_)) {
        ROS_INFO_STREAM("valTask2::alignSolarArrayTask : pose chaned to "<<solar_array_fine_walk_goal_);
        task2_utils_->taskLogPub("valTask2::alignSolarArrayTask : pose chaned to x: " + std::to_string(solar_array_fine_walk_goal_.x) + "y: " + std::to_string(solar_array_fine_walk_goal_.y) + " theta:" + std::to_string(solar_array_fine_walk_goal_.theta));
        walker_->walkToGoal(solar_array_fine_walk_goal_, false);
        // sleep so that the walk starts
        ROS_INFO("valTask2::alignSolarArrayTask : Footsteps should be generated now");
        task2_utils_->taskLogPub("valTask2::alignSolarArrayTask : Footsteps should be generated now");
        ros::Duration(4).sleep();
        // update the previous pose
        pose_prev = solar_array_fine_walk_goal_;
        eventQueue.riseEvent("/ALIGN_TO_ARRAY_EXECUTING");
    }
    // if walking stay in the same state
    else if (walk_track_->isWalking())
    {
        // no state change
        ROS_INFO_THROTTLE(2, "valTask2::alignSolarArrayTask : walking");
        eventQueue.riseEvent("/ALIGN_TO_ARRAY_EXECUTING");
    }
    // if walk finished
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        geometry_msgs::Pose2D temp;
        pose_prev = temp;
        ROS_INFO("valTask2::alignSolarArrayTask : walk failed");
        task2_utils_->taskLogPub("valTask2::alignSolarArrayTask : walk failed");
        executeOnce = true;
        eventQueue.riseEvent("/ALIGN_TO_ARRAY_FAILED");
    }
    // if failed retry detecting the array and then walk
    else
    {
        // increment the fail count
        fail_count++;
        geometry_msgs::Pose2D temp;
        pose_prev = temp;
        ROS_INFO("valTask2::alignSolarArrayTask : walk retry");
        task2_utils_->taskLogPub("valTask2::alignSolarArrayTask : walk retry");
        eventQueue.riseEvent("/ALIGN_TO_ARRAY_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::alignSolarArrayTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::alignSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::placePanelTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::placePanelTask : executing " + name);

    if(skip_4)
    {
        // go to next state of detecting cable
        ROS_INFO("valTask2::placePanelTask: [SKIP] skipping place Panel Task state");
        eventQueue.riseEvent("/PLACED_ON_GROUND");
        return TaskResult::SUCCESS();
    }

    static bool handsPulledOff = false;

    /************************************
     *  Get into panel placement pose
     *  move down till effort reduces
     *  open grippers
     *  pull hand back
     ************************************
     */
    if (task2_utils_->isPanelPicked(panel_grasping_hand_)){
        ROS_INFO("valTask2::placePanelTask : Placing the panel on table");
        task2_utils_->taskLogPub("valTask2::placePanelTask : Placing the panel on table");
        task2_utils_->moveToPlacePanelPose(panel_grasping_hand_, is_rotation_required_);
        ros::Duration(1).sleep();
        eventQueue.riseEvent("/PLACE_ON_GROUND_RETRY");
    }
    else if (!handsPulledOff){

        geometry_msgs::Pose currentPalmPose;
        std::string palmFrame = panel_grasping_hand_ == armSide ::LEFT ? VAL_COMMON_NAMES::L_PALM_TF : VAL_COMMON_NAMES::R_PALM_TF;

        robot_state_->getCurrentPose(palmFrame, currentPalmPose, VAL_COMMON_NAMES::PELVIS_TF);
        currentPalmPose.position.x -= 0.2;
        robot_state_->transformPose(currentPalmPose, currentPalmPose, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

        arm_controller_->moveArmInTaskSpace(panel_grasping_hand_, currentPalmPose, 1.0f);
        ros::Duration(1).sleep();

        arm_controller_->moveToZeroPose((armSide)!panel_grasping_hand_);
        ros::Duration(0.5).sleep();
        arm_controller_->moveToZeroPose(panel_grasping_hand_);
        ros::Duration(0.5).sleep();

        handsPulledOff = true;
        eventQueue.riseEvent("/PLACE_ON_GROUND_RETRY");
    }
    else if (task2_utils_->getCurrentCheckpoint() > 2){
        ROS_INFO("valTask2::placePanelTask : Placed the panel successfully");
        task2_utils_->taskLogPub("valTask2::placePanelTask : Placed the panel successfully");
        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(1).sleep();
        eventQueue.riseEvent("/PLACED_ON_GROUND");
    }
    else {
        ROS_INFO("valTask2::placePanelTask : Panel placement failed");
        task2_utils_->taskLogPub("valTask2::placePanelTask : Panel placement failed");
        eventQueue.riseEvent("/PLACED_ON_GROUND_FAILED");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::deployPanelTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectButtonTask: executing " << name);
    task2_utils_->taskLogPub("valTask2::detectButtonTask: executing " + name);

    if(skip_4)
    {
        // go to next state of detecting cable
        ROS_INFO("valTask2::detectButtonTask: [SKIP] skipping detecting button state");
        eventQueue.riseEvent("/BUTTON_DETECTED");
        return TaskResult::SUCCESS();
    }

    static std::queue<float> head_yaw_ranges;

    static bool executingOnce = true;
    if(executingOnce)
    {
        head_controller_->moveHead(0,30,0,1.5);
        ros::Duration(2).sleep();

        // possible head yaw rotations to better detect the cable
        //clear the queue before starting
        std::queue<float> temp;
        head_yaw_ranges= temp;
        head_yaw_ranges.push( 55.0);
        head_yaw_ranges.push( 45.0);
        head_yaw_ranges.push( 35.0);
        head_yaw_ranges.push( 20.0);
        head_yaw_ranges.push( 10.0);
        head_yaw_ranges.push(-10.0);
        head_yaw_ranges.push(-20.0);
        head_yaw_ranges.push(-35.0);
        head_yaw_ranges.push(-45.0);
        head_yaw_ranges.push(-55.0);

        executingOnce = false;
        control_common_->stopAllTrajectories();
    }


    int retry = 0;
    while (!button_detector_->findButtons(button_coordinates_) && retry++ < 5);
    std::cout<<"valTask2::detectButtonTask : retry count : "<<retry<<"\n";
    std::cout<<"button x: "<<button_coordinates_.x<<" button y: "<<button_coordinates_.y<<" button z: "<<button_coordinates_.z<<"\n";

    if(button_coordinates_.x == 0 && !head_yaw_ranges.empty())
    {
        // wrong point detected. move head and try again
        head_controller_->moveHead(0,30,head_yaw_ranges.front(), 2.0f);
        ROS_INFO("valTask2::detectButtonTask : Button not found. Trying with %.6f angle",head_yaw_ranges.front());
        task2_utils_->taskLogPub("valTask2::detectButtonTask : Button not found. Trying with " + std::to_string(head_yaw_ranges.front()) +  " angle");
        head_yaw_ranges.pop();
        ros::Duration(2.5).sleep();
        control_common_->stopAllTrajectories();
        eventQueue.riseEvent("/DETECT_BUTTON_RETRY");
    }
    else if (button_coordinates_.x == 0)
    {
        // state failed after detecting 5*4 times
        ROS_INFO("valTask2::detectButtonTask : Button not found. Failed after 20 trials");
        task2_utils_->taskLogPub("valTask2::detectButtonTask : Button not found. Failed after 20 trials");
        eventQueue.riseEvent("/BUTTON_DETECTION_FAILED");
        executingOnce= true;
        head_controller_->moveHead(0,0,0,2.0f);

    }
    else
    {
        // button found

        std::cout<<"button x: "<<button_coordinates_.x<<" button y: "<<button_coordinates_.y<<" button z: "<<button_coordinates_.z<<"\n";
        eventQueue.riseEvent("/BUTTON_DETECTED");
        executingOnce= true;
        head_controller_->moveHead(0,0,0,2.0f);

        ///@todo use goal position rather than hard coded steps. determine after experimenation
        /// the robot is expected to stand atleast 0.15 meters to the right of the button to press it.

        task2_utils_->reOrientTowardsGoal(button_coordinates_,-0.25);
        ros::Duration(1).sleep();
    }


    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectSolarArrayTask : waiting for transition");
    }
    return TaskResult::SUCCESS();
}



decision_making::TaskResult valTask2::deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    if(skip_4)
    {
        // go to next state of detecting cable
        ROS_INFO("valTask2::deployPanelTask: [SKIP] skipping deploying panel state");
        eventQueue.riseEvent("/DEPLOYED");
        arm_controller_->moveToDefaultPose(armSide::LEFT);  /// to account for panel being very close to body
        ros::Duration(1).sleep();

        skip_4=false;
        return TaskResult::SUCCESS();
    }

    if(button_press_ == nullptr) {
        button_press_ = new ButtonPress(nh_);
    }

    static std::queue<geometry_msgs::Point> points;
    static bool executeOnce =true;
    if(executeOnce)
    {
        ROS_INFO("valTask2::deployPanelTask : setting up possible button locations for retry");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : setting up possible button locations for retry");
        //clear the queue before starting
        std::queue<geometry_msgs::Point> temp;
        points= temp;

        points.push(button_coordinates_);

        // define 4 retry points before going back to detection
        float offset=0.08;
        geometry_msgs::Point buttonPelvis;
        robot_state_->transformPoint(button_coordinates_,buttonPelvis,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
        std::vector<geometry_msgs::Point> retryPoints;

        retryPoints.resize(4);
        // First Retry Point
        retryPoints[0]=buttonPelvis;
        retryPoints[0].x-=offset;
        robot_state_->transformPoint(retryPoints[0],retryPoints[0],VAL_COMMON_NAMES::PELVIS_TF);

        // Second Retry Point
        retryPoints[1]=buttonPelvis;
        retryPoints[1].x+=offset;
        robot_state_->transformPoint(retryPoints[1],retryPoints[1],VAL_COMMON_NAMES::PELVIS_TF);

        // Third Retry Point
        retryPoints[2]=buttonPelvis;
        retryPoints[2].y+=offset;
        robot_state_->transformPoint(retryPoints[2],retryPoints[2],VAL_COMMON_NAMES::PELVIS_TF);

        // Fourth Retry Point
        retryPoints[3]=buttonPelvis;
        retryPoints[3].y-=offset;
        robot_state_->transformPoint(retryPoints[3],retryPoints[3],VAL_COMMON_NAMES::PELVIS_TF);

        // pushing all points in queue
        points.push(retryPoints[0]);
        points.push(retryPoints[1]);
        points.push(retryPoints[2]);
        points.push(retryPoints[3]);

        // setting executing once to false to ensure it does not enter this loop again unless a new detection comes in.
        executeOnce=false;
    }

    static int retry_count=0;

    if(task2_utils_->getCurrentCheckpoint() > 3)
    {
        ROS_INFO("valTask2::deployPanelTask : Panel deployed successfully");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : Panel deployed successfully");
        eventQueue.riseEvent("/DEPLOYED");
        task2_utils_->clearPointCloud();
        ros::Duration(1).sleep();

        if (button_press_ != nullptr) delete button_press_;
        button_press_ = nullptr;
    }
    else if(!points.empty())
    {
        ROS_INFO("valTask2::deployPanelTask : Reaching out to the button");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : Reaching out to the button");
        // choosing default side to be left at the moment. it might work with right hand as well.
        button_press_->pressButton(LEFT,points.front());
        points.pop();
        executeOnce=false;
        eventQueue.riseEvent("/DEPLOY_RETRY");
    }
    else if(points.empty() && retry_count <5)
    {
        ROS_INFO("valTask2::deployPanelTask : deployment failed for all 5 points. retrying");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : deployment failed for all 5 points. retrying");
        retry_count++;
        executeOnce=true;
        eventQueue.riseEvent("/DETECT_AGAIN");
        if (button_press_ != nullptr) delete button_press_;
        button_press_ = nullptr;
    }
    else if(retry_count >5)
    {
        ROS_INFO("valTask2::deployPanelTask : Failed 20 times. time to exit");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : Failed 20 times. time to exit");
        executeOnce=true;
        eventQueue.riseEvent("/DEPLOY_FAILED");
        if (button_press_ != nullptr) delete button_press_;
        button_press_ = nullptr;
    }
    else
    {
        ROS_WARN("DEBUG the state : deployPanelTask. retried %d times", retry_count);
        eventQueue.riseEvent("/DEPLOY_FAILED");
        executeOnce=true;
        if (button_press_ != nullptr) delete button_press_;
        button_press_ = nullptr;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::deployPanelTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::deployPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("valTask2::detectCableTask: executing " << name);
    task2_utils_->taskLogPub("valTask2::detectCableTask: executing " + name);

    static std::queue<float> head_yaw_ranges;

    static bool executingOnce = true;
    if(executingOnce)
    {
        head_controller_->moveHead(0,40,-30,1.5);
        ros::Duration(2).sleep();

        // possible head yaw rotations to better detect the cable
        //clear the queue before starting
        std::queue<float> temp;
        head_yaw_ranges= temp;
        head_yaw_ranges.push(-20.0);
        head_yaw_ranges.push(-10.0);
        head_yaw_ranges.push( 0.0);
        head_yaw_ranges.push( 20.0);
        executingOnce = false;
        control_common_->stopAllTrajectories();
    }


    int retry = 0;
    while (!cable_detector_->findCable(cable_pose_) && retry++ < 5);
    std::cout<<"valTask2::detectCableTask: retry count : "<<retry<<"\n";
    std::cout<<"cable x: "<<cable_pose_.position.x<<" cable y: "<<cable_pose_.position.y<<" cable z: "<<cable_pose_.position.z<<"\n";

    if(cable_pose_.position.x == 0 && !head_yaw_ranges.empty())
    {
        // wrong point detected. move head and try again
        head_controller_->moveHead(0,40,head_yaw_ranges.front(), 2.0f);
        ROS_INFO("valTask2::detectCableTask: Cable not found. Trying with %.6f angle",head_yaw_ranges.front());
        task2_utils_->taskLogPub("valTask2::detectCableTask: Cable not found. Trying with" + std::to_string(head_yaw_ranges.front()) + " angle");
        head_yaw_ranges.pop();
        ros::Duration(2.5).sleep();
        control_common_->stopAllTrajectories();
        eventQueue.riseEvent("/DETECT_CABLE_RETRY");
    }
    else if (cable_pose_.position.x == 0)
    {
        // state failed after detecting 5*4 times
        ROS_INFO("valTask2::detectCableTask : Cable not found. Failed after 20 trials");
        task2_utils_->taskLogPub("valTask2::detectCableTask : Cable not found. Failed after 20 trials");
        eventQueue.riseEvent("/DETECT_CABLE_FAILED");
        executingOnce= true;
        head_controller_->moveHead(0,0,0,2.0f);

    }
    else
    {
        // cable found

        std::cout<<"cable x: "<<cable_pose_.position.x<<" cable y: "<<cable_pose_.position.y<<" cable z: "<<cable_pose_.position.z<<"\n";
        eventQueue.riseEvent("/DETECTED_CABLE");
        executingOnce= true;
        task2_utils_->reOrientTowardsGoal(cable_pose_.position,0.4);
        head_controller_->moveHead(0,0,0,2.0f);
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSocketTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectSocketTask : waiting for transition");
    }
    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::pickCableTask : executing " << name);
    task2_utils_->taskLogPub("valTask2::pickCableTask : executing " + name);

    /*
     * isCableOnTable   -- pickup the cable and retry
     * isCableInHand    -- task complete continue
     * else - fail
     */
    static int retry = 0;
    static bool executeOnce= true;
    if(executeOnce)
    {
        pelvis_controller_->controlPelvisHeight(0.85);
        ros::Duration(0.3).sleep();
        executeOnce=false;
    }

    if(cable_task_ == nullptr)
    {
        cable_task_ = new CableTask(nh_);
        ROS_INFO("valTask2::pickCableTask : Initializing object");
        task2_utils_->taskLogPub("valTask2::pickCableTask : Initializing object");
    }

    if (task2_utils_->isCableOnTable(cable_pose_)){

        ROS_INFO("valTask2::pickCableTask : Picking up the cable");
        task2_utils_->taskLogPub("valTask2::pickCableTask : Picking up the cable");
        ///@todo add yaw condition here. if yaw greater then certain angle, it should rotate first, then grasp
        cable_task_->grasp_choke(armSide::RIGHT,cable_pose_);
        ros::Duration(1).sleep();
    }

    //always use right hand for cable pickup
    //if(task2_utils_->getCurrentCheckpoint() > 4 && task2_utils_->isCableInHand(armSide::RIGHT))
    // use above function when isCableInHand is populated

    if(task2_utils_->getCurrentCheckpoint() > 4)
    {
        ROS_INFO("valTask2::pickCableTask : Picked up cable");
        task2_utils_->taskLogPub("valTask2::pickCableTask : Picked up cable");
        // go to next state
        eventQueue.riseEvent("/CABLE_PICKED");
        pelvis_controller_->controlPelvisHeight(0.90);
        ros::Duration(0.3).sleep();
        executeOnce=true;

    }
    else if(retry <15)
    {
        ROS_INFO("valTask2::pickCableTask : Failed picking up the cable. Retrying detection");
        task2_utils_->taskLogPub("valTask2::pickCableTask : Failed picking up the cable. Retrying detection");
        retry++;
        // drop cable motion here
        eventQueue.riseEvent("/PICKUPCABLE_RETRY");
    }
    else
    {
        ROS_INFO("valTask2::pickCableTask : Failed picking the cable.");
        task2_utils_->taskLogPub("valTask2::pickCableTask : Failed picking the cable.");
        if (cable_detector_ != nullptr) delete cable_detector_;
        cable_detector_ = nullptr;
        pelvis_controller_->controlPelvisHeight(0.90);
        ros::Duration(0.3).sleep();
        executeOnce=true;
        eventQueue.riseEvent("/PICKUP_CABLE_FAILED");
    }

    // generate the event
    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::pickCableTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::pickCableTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

TaskResult valTask2::detectSocketTask(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectSocketTask: executing " << name);
    task2_utils_->taskLogPub("valTask2::detectSocketTask: executing " + name);

    static std::queue<float> head_yaw_ranges;

    static bool executingOnce = true;
    if(executingOnce)
    {
        head_controller_->moveHead(0,40,30,1.5);
        ros::Duration(2).sleep();

        // possible head yaw rotations to better detect the cable
        //clear the queue before starting
        std::queue<float> temp;
        head_yaw_ranges= temp;
        head_yaw_ranges.push( 20.0);
        head_yaw_ranges.push( 10.0);
        head_yaw_ranges.push( 0.0);
        head_yaw_ranges.push( 10.0);
        head_yaw_ranges.push( 20.0);
        executingOnce = false;
        control_common_->stopAllTrajectories();
        //        walker_->walkLocalPreComputedSteps({-0.1,-0.1},{0.0,0.0},RIGHT);
        ros::Duration(2).sleep();
    }


    int retry = 0;
    while (!socket_detector_->findPlug(socket_coordinates_) && retry++ < 5);
    std::cout<<"valTask2::detectSocketTask:: retry count : "<<retry<<"\n";
    std::cout<<"socket x: "<<socket_coordinates_.x<<" socket y: "<<socket_coordinates_.y<<" socket z: "<<socket_coordinates_.z<<"\n";

    if(socket_coordinates_.x == 0 && !head_yaw_ranges.empty())
    {
        // wrong point detected. move head and try again
        head_controller_->moveHead(0,30,head_yaw_ranges.front(), 2.0f);
        ROS_INFO("valTask2::detectSocketTask: Socket not found. Trying with %.6f angle",head_yaw_ranges.front());
        task2_utils_->taskLogPub("valTask2::detectSocketTask: Socket not found. Trying with " + std::to_string(head_yaw_ranges.front()) + " angle");
        head_yaw_ranges.pop();
        ros::Duration(2.5).sleep();
        control_common_->stopAllTrajectories();
        eventQueue.riseEvent("/DETECT_SOCKET_RETRY");
    }
    else if (socket_coordinates_.x == 0)
    {
        // state failed after detecting 5*4 times
        ROS_INFO("valTask2::detectSocketTask : Socket not found. Failed after 20 trials");
        task2_utils_->taskLogPub("valTask2::detectSocketTask : Socket not found. Failed after 20 trials");
        head_controller_->moveHead(0,0,0,2.0f);
        eventQueue.riseEvent("/DETECT_SOCKET_FAILED");
        executingOnce= true;

    }
    else
    {
        // socket found

        std::cout<<"socket x: "<<socket_coordinates_.x<<" socket y: "<<socket_coordinates_.y<<" socket z: "<<socket_coordinates_.z<<"\n";
        eventQueue.riseEvent("/DETECTED_SOCKET");
        executingOnce= true;
        head_controller_->moveHead(0,0,0,2.0f);
        //        walker_->walkLocalPreComputedSteps({0.1,0.1},{0.0,0.0},RIGHT);
        ros::Duration(2.5).sleep();
        task2_utils_->reOrientTowardsGoal(socket_coordinates_);
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSocketTask : waiting for transition");
        task2_utils_->taskLogPub("valTask2::detectSocketTask : waiting for transition");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    //    if (cable_task_ != nullptr) delete cable_task_;
    //    cable_task_ = nullptr;

    static int retry=0;
    static int fail =0;

    if(cable_task_ == nullptr)
    {
        cable_task_ = new CableTask(nh_);
        ROS_INFO("valTask2::pickCableTask : Initializing object");
    }

    //    Checkpoint 5: Plug the power cable into the solar panel
    if(cable_task_->insert_cable(socket_coordinates_) && task2_utils_->getCurrentCheckpoint() > 5)
    {
        ROS_INFO("valTask2::plugCableTask plugged the cable succesfully. going to next state");
        task2_utils_->taskLogPub("valTask2::plugCableTask plugged the cable succesfully. going to next state");
        eventQueue.riseEvent("/CABLE_PLUGGED");
    }
    else if(task2_utils_->isCableInHand(armSide::RIGHT) && retry <5)
    {
        ROS_INFO("valTask2::plugCableTask cable is in hand. attempting to retry plugging the cable");
        task2_utils_->taskLogPub("valTask2::plugCableTask cable is in hand. attempting to retry plugging the cable");
        retry++;
        eventQueue.riseEvent("/PLUGIN_CABLE_RETRY");
    }
    else if(fail <5)
    {
        ///@todo function to drop cable at seed point
        ROS_INFO("valTask2::plugCableTask retried 5 times. going back to detect the cable");
        task2_utils_->taskLogPub("valTask2::plugCableTask retried 5 times. going back to detect the cable");
        fail++;
        retry=0;
        eventQueue.riseEvent("/REDETECT_CABLE");
    }
    else
    {
        ROS_INFO("valTask2::plugCableTask failed all attempts. going to error state");
        task2_utils_->taskLogPub("valTask2::plugCableTask failed all attempts. going to error state");
        eventQueue.riseEvent("/PLUGIN_CABLE_FAILED");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectFinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectFinishBoxTask :executing " << name);
    task2_utils_->taskLogPub("valTask2::detectFinishBoxTask :executing " + name);

    static int retry_count = 0;
    // generate the event
    if (finish_box_detector_ == nullptr){
        finish_box_detector_ = new FinishBoxDetector(nh_);
        ros::Duration(2).sleep();   // map is updated every 2 sec
    }

    std::vector<geometry_msgs::Point> detections;
    if(visited_map_.data.empty()){
        retry_count++;
        ROS_INFO("valTask2::detectFinishBoxTask: visited map is empty");
        task2_utils_->taskLogPub("valTask2::detectFinishBoxTask: visited map is empty");
        ros::Duration(3).sleep();   // this will execute only when visited map is not updated which should never happen
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    } else  if(finish_box_detector_->getFinishBoxCenters(detections)){
        for(size_t i = 0; i < detections.size(); ++i){
            size_t index = MapGenerator::getIndex(detections[i].x, detections[i].y);
            ROS_INFO("Index in map %d and size of visited map is %d", (int)index, (int)visited_map_.data.size());
            task2_utils_->taskLogPub("Index in map " + std::to_string((int)index) + " and size of visited map is " + std::to_string((int)visited_map_.data.size()));
            if(visited_map_.data.at(index) == CELL_STATUS::VISITED){
                continue;
            }

            next_finishbox_center_.x = detections[i].x;
            next_finishbox_center_.y = detections[i].y;
            next_finishbox_center_.theta = atan2(next_finishbox_center_.y, next_finishbox_center_.x);
            ROS_INFO("detectFinishBoxTask: Successful");
            task2_utils_->taskLogPub("detectFinishBoxTask: Successful");
            eventQueue.riseEvent("/DETECT_FINISH_SUCESSFUL");
            retry_count=0;
            if(finish_box_detector_ != nullptr) delete finish_box_detector_;
            finish_box_detector_ = nullptr;
            //sleep is required to avoid moving to next state before subscriber is shutdown
            //            ros::Duration(2).sleep();
            break;
        }
        // this is to avoid detecting points that will always be in collision
        retry_count++;
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    }
    else if(retry_count++ < 15){
        ros::Duration(3).sleep();   // map is updated every 4 sec
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    }
    else{
        eventQueue.riseEvent("/DETECT_FINISH_FAILED");
        if(finish_box_detector_ != nullptr) delete finish_box_detector_;
        finish_box_detector_ = nullptr;
        retry_count=0;
        //sleep is required to avoid moving to next state before subscriber is shutdown
        //        ros::Duration(2).sleep();
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("detectfinishBoxTask: waiting for transition");
        task2_utils_->taskLogPub("detectfinishBoxTask: waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {

    ROS_INFO_STREAM("valTask2::walkToFinishTask : executing " << name);


    static bool execute_once = true;

    if (execute_once){
        task2_utils_->clearCurrentPoseMap();
        task2_utils_->taskLogPub("valTask2::walkToFinishTask : executing " + name);
        // set the robot to default state to walk
        gripper_controller_->openGripper(armSide::RIGHT);
        ros::Duration(0.2).sleep();
        gripper_controller_->openGripper(armSide::LEFT);
        ros::Duration(1).sleep();
        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(0.2).sleep();
        chest_controller_->controlChest(2, 2, 2);
        ros::Duration(0.2).sleep();
        arm_controller_->moveToDefaultPose(armSide::LEFT);
        ros::Duration(0.2).sleep();
        arm_controller_->moveToDefaultPose(armSide::RIGHT);
        ros::Duration(0.2).sleep();
        execute_once = false;
    }


    // kill the node and exit
    ROS_INFO("task2 completed killing the node");
    task2_utils_->taskLogPub("task2 completed killing the node");

    int ret = std::system("rosnode kill task2");

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
        task1_utils_->taskLogPub("Give manual walk goal");
    }

    return TaskResult::SUCCESS();

    static int fail_count = 0;

    // walk to the goal location

    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    // check if the pose is changed
    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, next_finishbox_center_)) {
        ROS_INFO("walkToFinishTask: reached panel");
        task2_utils_->taskLogPub("walkToFinishTask: reached panel");
        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/WALK_TO_FINISH_SUCESSFUL");

    }
    // the goal can be updated on the run time
    else if (taskCommonUtils::isPoseChanged(pose_prev, next_finishbox_center_))
    {
        ROS_INFO("walkToFinishTask: pose changed");
        task2_utils_->taskLogPub("walkToFinishTask: pose changed");
        walker_->walkToGoal(next_finishbox_center_, false);
        ROS_INFO("walkToFinishTask: Footsteps should be published now");
        task2_utils_->taskLogPub("walkToFinishTask: Footsteps should be published now");
        // sleep so that the walk starts
        ros::Duration(4).sleep();

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
        ROS_INFO("walkToFinishTask: walk failed");
        task2_utils_->taskLogPub("walkToFinishTask: walk failed");
        eventQueue.riseEvent("/WALK_TO_FINISH_ERROR");
    }
    // if failed retry detecting the panel and then walk
    // also handles MOVE_FAILED
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("walkToFinishTask: walk retry");
        task2_utils_->taskLogPub("walkToFinishTask: walk retry");
        eventQueue.riseEvent("/WALK_TO_FINISH_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("walkToFinishTask: waiting for transition");
        task2_utils_->taskLogPub("walkToFinishTask: waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::skipCheckPointTask: waiting for transition");
        task2_utils_->taskLogPub("valTask2:: error state: waiting for transition");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::skipCheckPointTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    // skip check point, basically take the user input and switches the state

    //    wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::skipCheckPointTask: waiting for transition");
        task2_utils_->taskLogPub("valTask2::skipCheckPointTask: waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::skipToCP3Task(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    static int retry_count = 0;

    // skip to checkpoint 3
    // start the task
    ros::ServiceClient  client = nh_.serviceClient<srcsim::StartTask>("/srcsim/finals/start_task");
    srcsim::StartTask   srv;
    srv.request.checkpoint_id  = 3;
    srv.request.task_id        = 2;


    // if the check point is skipped
    if(client.call(srv))
    {
        ///@TODO: do anything which is required for further states
        task2_utils_->checkpoint_init();
        skip_3=true;
        eventQueue.riseEvent("/SKIPPED_TO_CP_3");
    }
    else if(retry_count < 5)
    {
        //reset the count
        retry_count = 0;
        eventQueue.riseEvent("/SKIP_CP_3_RETRY");
    }
    else
    {
        ROS_ERROR("service not called");
        eventQueue.riseEvent("/SKIP_CP_3_FAILED");
        retry_count++;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("skipCheckPoint3Task: waiting for transition");
        task2_utils_->taskLogPub("skipCheckPoint3Task: waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::skipToCP4Task(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    static int retry_count = 0;

    // skip to checkpoint 4
    // start the task
    ros::ServiceClient  client = nh_.serviceClient<srcsim::StartTask>("/srcsim/finals/start_task");
    srcsim::StartTask   srv;
    srv.request.checkpoint_id  = 4;
    srv.request.task_id        = 2;

    // if the check point is skipped
    if(client.call(srv))
    {
        ///@TODO: do anything which is required for further states
        task2_utils_->checkpoint_init();
        skip_3=true;
        skip_4=true;
        eventQueue.riseEvent("/SKIPPED_TO_CP_4");
    }
    else if(retry_count < 5)
    {
        //reset the count
        retry_count = 0;
        eventQueue.riseEvent("/SKIP_CP_4_RETRY");
    }
    else
    {
        ROS_ERROR("service not called");
        eventQueue.riseEvent("/SKIP_CP_4_FAILED");
        retry_count++;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("skipCheckPoint5Task: waiting for transition");
        task2_utils_->taskLogPub("skipCheckPoint5Task: waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::skipToCP6Task(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    static int retry_count = 0;

    // skip to checkpoint 6
    // start the task
    ros::ServiceClient  client = nh_.serviceClient<srcsim::StartTask>("/srcsim/finals/start_task");
    srcsim::StartTask   srv;
    srv.request.checkpoint_id  = 6;
    srv.request.task_id        = 2;

    // if the check point is skipped
    if(client.call(srv))
    {
        ///@TODO: do anything which is required for further states
        task2_utils_->checkpoint_init();
        skip_6=true;
        ros::Duration(1).sleep();
        task2_utils_->clearCurrentPoseMap();
        ros::Duration(1).sleep();
        walker_->walk_rotate(1.57);
        ros::Duration(2).sleep();
        walker_->walk_rotate(1.57);
        ros::Duration(2).sleep();
        walker_->walk_rotate(1.57);
        ros::Duration(2).sleep();
        walker_->walk_rotate(1.57);
        ros::Duration(2).sleep();
        eventQueue.riseEvent("/SKIPPED_TO_CP_6");
    }
    else if(retry_count < 5)
    {
        //reset the count
        retry_count = 0;
        eventQueue.riseEvent("/SKIP_CP_6_RETRY");
    }
    else
    {
        ROS_ERROR("service not called");
        eventQueue.riseEvent("/SKIP_CP_6_FAILED");
        retry_count++;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("skipCheckPoint6Task: waiting for transition");
        task2_utils_->taskLogPub("skipCheckPoint6Task: waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::manualExecutionTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);
    task2_utils_->taskLogPub("executing " + name);

    // skip check point, basically take the user input
    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::manualExecutionTask: waiting for transition");
        task2_utils_->taskLogPub("valTask2::manualExecutionTask: waiting for transition");
    }

    return TaskResult::SUCCESS();
}

// setter and getter methods
geometry_msgs::Pose2D valTask2::getPanelWalkGoal()
{
    return panel_walk_goal_;
}

void valTask2::setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    panel_walk_goal_ = panel_walk_goal;
}

void valTask2::setRoverWalkGoal(const std::vector<geometry_msgs::Pose2D> &rover_walk_goal)
{
    rover_walk_goal_waypoints_ = rover_walk_goal;

}

void valTask2::setRoverSide(const bool isRoverOnRight)
{
    is_rover_on_right_ = isRoverOnRight;
}

void valTask2::setSolarPanelHandlePose(const geometry_msgs::Pose &pose)
{
    solar_panel_handle_pose_ = pose;
}

void valTask2::setSolarArrayWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    solar_array_walk_goal_ = panel_walk_goal;
}

void valTask2::setSolarArrayFineWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    solar_array_fine_walk_goal_ = panel_walk_goal;
}

void valTask2::setSolarArraySide(const bool isSolarArrayOnRight)
{
    is_array_on_right_ = isSolarArrayOnRight;
}

void valTask2::setPanelGraspingHand(armSide side)
{
    panel_grasping_hand_ = side;
}

void valTask2::setIsRotationRequired(bool value)
{
    is_rotation_required_ = value;
}

void valTask2::setTempCablePoint(geometry_msgs::Point value)
{
    cable_point_temp_ = value;
}
