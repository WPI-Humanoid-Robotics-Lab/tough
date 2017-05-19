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

using namespace std;

#define foreach BOOST_FOREACH

valTask2 *valTask2::currentObject = nullptr;

valTask2* valTask2::getValTask2(ros::NodeHandle nh){
    if(currentObject == nullptr){
        currentObject = new valTask2(nh);
        return currentObject;
    }
    ROS_ERROR("Object already exists");
    assert(false && "Object already exists");
}


// constructor and destrcutor
valTask2::valTask2(ros::NodeHandle nh):
    nh_(nh)
{
    // object for the valkyrie walker
    walker_ = new ValkyrieWalker(nh_, 0.7, 0.7, 0, 0.18);
    pelvis_controller_ = new pelvisTrajectory(nh_);
    walk_track_ = new walkTracking(nh_);
    //initialize all detection pointers
    rover_detector_ = nullptr;
    solar_panel_detector_ = nullptr;
    rover_in_map_blocker_ = nullptr;
    panel_grabber_        = nullptr;

    //utils
    task2_utils_    = new task2Utils(nh_);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    map_update_count_ = 0;
    occupancy_grid_sub_ = nh_.subscribe("/map",10, &valTask2::occupancy_grid_cb, this);
}

// destructor
valTask2::~valTask2(){
    delete walker_;
}

void valTask2::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){
    ++map_update_count_;
}

bool valTask2::preemptiveWait(double ms, decision_making::EventQueue& queue) {
    for (int i = 0; i < 100 && !queue.isTerminated(); i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

    return queue.isTerminated();
}

// This functions are called based on the remaping in the main.
// when ever a action is published one of these functions will be called
decision_making::TaskResult valTask2::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

    ROS_INFO_STREAM("executing " << name);
    static int retry_count = 0;

    // if the map does not update fast enought and this is called greater then 10 time it will break
    if(retry_count == 0){
        map_update_count_ = 0;
        taskCommonUtils::moveToInitPose(nh_);
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
        srv.request.task_id       = 2;
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

decision_making::TaskResult valTask2::detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    if(rover_detector_ == nullptr){
        rover_detector_ = new RoverDetector(nh_);
    }

    static int fail_count = 0;
    static int retry_count = 0;
    std::vector<geometry_msgs::Pose> poses;
    rover_detector_->getDetections(poses);

    ROS_INFO("Size of poses : %d", (int)poses.size());
    //    // if we get atleast one detection, LOL
    if (poses.size() > 1) {
        // update the pose
        geometry_msgs::Pose2D pose2D;
        // get the last detected pose
        int idx = poses.size() -1 ;
        pose2D.x = poses[idx].position.x;
        pose2D.y = poses[idx].position.y;

        std::cout << "x " << pose2D.x << " y " << pose2D.y << std::endl;

        // get the theta
        pose2D.theta = tf::getYaw(poses[idx].orientation);
        setRoverWalkGoal(pose2D);
        ROVER_SIDE roverSide;
        if(rover_detector_->getRoverSide(roverSide)){
            setRoverSide(roverSide == ROVER_SIDE::RIGHT);
            // block rover in /map
            rover_in_map_blocker_ = new SolarArrayDetector(nh_, pose2D,is_rover_on_right_);
            //wait for the map to update. This is required to ensure the footsteps dont collide with rover
            //        ros::Duration(0.5).sleep();
            taskCommonUtils::moveToWalkSafePose(nh_);

            std::cout << "quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w <<std::endl;
            std::cout << "yaw: " << pose2D.theta  <<std::endl;
            retry_count = 0;
            // update the plane coeffecients
            eventQueue.riseEvent("/DETECTED_ROVER");
            ROS_INFO("detected rover");
            if(rover_detector_ != nullptr) delete rover_detector_;
            rover_detector_ = nullptr;
        }
        else{
            eventQueue.riseEvent("/DETECT_ROVER_RETRY");
        }
    }
    else if(retry_count < 5) {
        ROS_INFO("sleep for 3 seconds for panel detection");
        ++retry_count;
        ros::Duration(2).sleep();
        eventQueue.riseEvent("/DETECT_ROVER_RETRY");
    }

    // if failed for more than 5 times, go to error state
    else if (fail_count > 5) {
        // reset the fail count
        ROS_INFO("Rover detection failed");
        fail_count = 0;
        eventQueue.riseEvent("/DETECT_ROVER_FAILED");
        if(rover_detector_ != nullptr) delete rover_detector_;
        rover_detector_ = nullptr;
    }
    // if failed retry detecting the panel
    else
    {
        ROS_INFO("Retrying detection");
        // increment the fail count
        fail_count++;
        eventQueue.riseEvent("/DETECT_ROVER_RETRY");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM_ONCE("executing " << name);

    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, rover_walk_goal_) ) {
        ROS_INFO("reached panel");

        ros::Duration(1).sleep();
        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/REACHED_ROVER");
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, rover_walk_goal_)) {
        ROS_INFO_STREAM("pose chaned to "<<rover_walk_goal_);
        walker_->walkToGoal(rover_walk_goal_, false);
        // sleep so that the walk starts
        ROS_INFO("Footsteps should be generated now");
        ros::Duration(4).sleep();
        // update the previous pose
        pose_prev = rover_walk_goal_;
    }
    // if walking stay in the same state
    if (walk_track_->isWalking())
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

decision_making::TaskResult valTask2::detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    if(solar_panel_detector_ == nullptr) {
        solar_panel_detector_ = new SolarPanelDetect(nh_, rover_walk_goal_, is_rover_on_right_);
        ros::Duration(0.2).sleep();
    }

    static int fail_count = 0;
    static int retry_count = 0;

    // detect solar panel
    std::vector<geometry_msgs::Pose> poses;
    solar_panel_detector_->getDetections(poses);

    // if we get atleast two detections
    if (poses.size() > 1)
    {
        size_t idx = poses.size()-1;
        setSolarPanelHandlePose(poses[idx]);

        std::cout << "Position " << poses[idx].position.x<< " " <<poses[idx].position.y <<" "<<poses[idx].position.z<<std::endl;
        std::cout << "quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w <<std::endl;
        retry_count = 0;
        eventQueue.riseEvent("/DETECTED_PANEL");
    }

    else if(retry_count < 5) {
        ROS_INFO("sleep for 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_PANEL_RETRY");
        ros::Duration(3).sleep();
    }
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        eventQueue.riseEvent("/DETECT_PANEL_FAILED");
        if(solar_panel_detector_ != nullptr) delete solar_panel_detector_;
        solar_panel_detector_ = nullptr;
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

decision_making::TaskResult valTask2::graspPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
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
    static armSide side;
    if(panel_grabber_ == nullptr){
        panel_grabber_ = new solar_panel_handle_grabber(nh_);
    }

    if(!executing){
        ROS_INFO("Executing the grasp panel handle command");
        executing = true;
        // grasp the handle
        side = is_rover_on_right_ ? armSide::RIGHT : armSide::LEFT;
        panel_grabber_->grasp_handles(side, solar_panel_handle_pose_);
        ros::Duration(0.2).sleep(); //wait till grasp is complete
        eventQueue.riseEvent("/GRASP_RETRY");
    }
    else if (robot_state_->isGraspped(side)){
        ROS_INFO("Grasp is successful");
        task2_utils_->afterPanelGraspPose(side);
        eventQueue.riseEvent("/GRASPED_PANEL");
    }
    else if (retry_count < 5 ){
        ROS_INFO("Grasp Failed, retrying");
        executing = false;
        ++retry_count;
        eventQueue.riseEvent("/GRASP_RETRY");
    }
    else {
        ROS_INFO("Failed all conditions. state error");
        executing = false;
        retry_count = 0;
        eventQueue.riseEvent("/GRASP_PANEL_FAILED");
    }

    //    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::dtectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    eventQueue.riseEvent("/REACHED_ROVER");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/WALK_TO_END");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/STOP_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

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

void valTask2::setRoverWalkGoal(const geometry_msgs::Pose2D &rover_walk_goal)
{
    rover_walk_goal_ = rover_walk_goal;
}

void valTask2::setRoverSide(const bool isRoverOnRight)
{
    is_rover_on_right_ = isRoverOnRight;
}

void valTask2::setSolarPanelHandlePose(const geometry_msgs::Pose &pose)
{
    solar_panel_handle_pose_ = pose;
}
