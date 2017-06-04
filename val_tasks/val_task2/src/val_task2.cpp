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
    button_detector_            = nullptr;
    button_press_               = nullptr;
    cable_task_                 = nullptr;
    cable_detector_             = nullptr;
    socket_detector_            = nullptr;
    finish_box_detector_        = nullptr;

    //utils
    task2_utils_    = new task2Utils(nh_);
    robot_state_    = RobotStateInformer::getRobotStateInformer(nh_);

    // Variables
    map_update_count_ = 0;
    is_rotation_required_ = false;
    panel_grasping_hand_ = armSide::RIGHT;
    // Subscribers
    occupancy_grid_sub_ = nh_.subscribe("/map",10, &valTask2::occupancy_grid_cb, this);
    visited_map_sub_    = nh_.subscribe("/visited_map",10, &valTask2::visited_map_cb, this);
}

// destructor
valTask2::~valTask2(){
    delete walker_;
    delete pelvis_controller_;
    delete walk_track_;
    delete chest_controller_;
    delete arm_controller_;
}

void valTask2::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){
    // Count the number of times map is updated
    ++map_update_count_;
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
    static int retry_count = 0;

    // reset mapcount when retrying the state for the first time
    if(retry_count == 0){
        map_update_count_ = 0;
        taskCommonUtils::moveToInitPose(nh_);
    }

    // the state transition can happen from an event externally or can be geenerated here
    ROS_INFO("valTask2::initTask : Occupancy Grid has been updated %d times, tried %d times", map_update_count_, retry_count);
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
        else
        {
            ROS_ERROR("service not called");
            eventQueue.riseEvent("/INIT_FAILED");
        }
        // generate the event
        eventQueue.riseEvent("/INIT_SUCESSFUL");

    }
    else if (map_update_count_ < 2 && retry_count++ < 40) {
        ROS_INFO("valTask2::initTask : Wait for occupancy grid to be updated with atleast 2 messages");
        ros::Duration(2.0).sleep();
        eventQueue.riseEvent("/INIT_RETRY");
    }
    else {
        retry_count = 0;
        ROS_INFO("valTask2::initTask : Failed to initialize");
        eventQueue.riseEvent("/INIT_FAILED");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::deployPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectRoverTask : executing " << name);

    if(rover_detector_ == nullptr){
        rover_detector_ = new RoverDetector(nh_);
    }

    static int retry_count = 0;
    static int fail_count = 0;
    std::vector<std::vector<geometry_msgs::Pose> > roverPoseWaypoints;
    rover_detector_->getDetections(roverPoseWaypoints);


    ROS_INFO("valTask2::detectRoverTask : Size of poses : %d", (int)roverPoseWaypoints.size());

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
        eventQueue.riseEvent("/DETECT_ROVER_FAILED");
        if(rover_detector_ != nullptr) delete rover_detector_;
        rover_detector_ = nullptr;
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
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
        for (auto pose : rover_walk_goal_waypoints_){
            ROS_INFO("X: %.2f  Y:%.2f  Theta:%.2f", pose.x, pose.y, pose.theta);
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
        // if coarse goal was already reached before, then the current state is fine goal reached, else it is coarse goal reached
        if(goal_waypoints.empty()){
            ROS_INFO("reached The rover");
            ros::Duration(3).sleep(); // This is required for steps to complete
            ROS_INFO("Final few steps before we reach rover");
            eventQueue.riseEvent("/REACHED_ROVER");
            executeOnce = true;
            ros::Duration(1).sleep();
        }
        else
        {
            ROS_INFO("%d more waypoints to go", goal_waypoints.size());
            goal= goal_waypoints.front();
            goal_waypoints.pop();
            ros::Duration(3).sleep();
            eventQueue.riseEvent("/WALK_EXECUTING");
        }
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, goal)) {
        ROS_INFO_STREAM("pose chaned to "<<goal);
        walker_->walkToGoal(goal, false);
        // sleep so that the walk starts
        ROS_INFO("Footsteps should be generated now");
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
        eventQueue.riseEvent("/WALK_FAILED");
    }
    // if failed retry detecting the panel and then walk
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
    ROS_INFO_STREAM("valTask2::detectPanelTask : executing " << name);
    static bool isFirstRun = true;

    if(isFirstRun){
        head_controller_->moveHead(0,40,0);
        ros::Duration(2).sleep();
    }
    if(solar_panel_detector_ == nullptr) {
        if (!isFirstRun){
            ROS_INFO("valTask2::detectPanelTask : Clearing pointcloud");
            task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::LARGE_BOX);
        }

        task2_utils_->resumePointCloud();
        isFirstRun = false;

        solar_panel_detector_ = new SolarPanelDetect(nh_, rover_walk_goal_waypoints_.back(), is_rover_on_right_);
        chest_controller_->controlChest(0, 0, 0);

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
    solar_panel_detector_->getDetections(poses);
    ROS_INFO("Size of detections : %d", poses.size());
    // if we get atleast two detections
    if (poses.size() > 1)
    {
        if (button_detector_ == nullptr){
            button_detector_ = new ButtonDetector(nh_);
        }
        size_t idx = poses.size()-1;
        setSolarPanelHandlePose(poses[idx]);
        task2_utils_->reOrientTowardsPanel(solar_panel_handle_pose_);
        ros::Duration(2.0).sleep();

        int retry = 0;
        while (!button_detector_->findButtons(button_coordinates_temp_) && retry++ < 10);
        ROS_INFO("valTask2::detectPanelTask : Button detected at x:%f y:%f z:%f", button_coordinates_temp_.x,button_coordinates_temp_.y,button_coordinates_temp_.z);
        ROS_INFO_STREAM("valTask2::detectPanelTask : Position " << poses[idx].position.x<< " " <<poses[idx].position.y <<" "<<poses[idx].position.z);
        ROS_INFO_STREAM("valTask2::detectPanelTask : quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w);
        retry_count = 0;

        eventQueue.riseEvent("/DETECTED_PANEL");
        if(solar_panel_detector_ != nullptr) delete solar_panel_detector_;
        solar_panel_detector_ = nullptr;

        if(button_detector_ != nullptr) delete button_detector_;
        button_detector_ = nullptr;
    }

    else if(retry_count < 10) {
        ROS_INFO("sleep for 3 seconds for panel detection");
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

        if(button_detector_ != nullptr) delete button_detector_;
        button_detector_ = nullptr;
    }


    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::graspPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::graspPanelTask : executing " << name);
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

    if(retry_count< 20){
        setSolarPanelHandlePose(task2_utils_->grasping_hand(hand,solar_panel_handle_pose_));
        setPanelGraspingHand(hand);
        is_rotation_required_=task2_utils_->isRotationReq(hand,solar_panel_handle_pose_.position,button_coordinates_temp_);

        ROS_INFO("valTask2::graspPanelTask : Rotation is %d ", is_rotation_required_);

        //Pause the pointcloud to avoid obstacles on map due to panel
        task2_utils_->pausePointCloud();
        retry_count++;

        if (panel_grabber_->grasp_handles(hand, solar_panel_handle_pose_)) {
            ROS_INFO("valTask2::graspPanelTask : Plan is 100 % Maybe Grasp is successful. Going to Pick Pannel Task");
            eventQueue.riseEvent("/GRASPED_PANEL");
        }
        else
        {
            ROS_INFO("valTask2::graspPanelTask :Redecting panel because plan was not 100 %. retry count = %d",retry_count);
            eventQueue.riseEvent("/REDETECT_PANEL");
        }
    }
    else
    {
        ROS_INFO("valTask2::graspPanelTask :All conditions failed. retry count = %d",retry_count);
        eventQueue.riseEvent("/GRASP_PANEL_FAILED");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::graspPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::pickPanelTask : executing " << name);

    task2_utils_->afterPanelGraspPose(panel_grasping_hand_);

    if (task2_utils_->isPanelPicked(panel_grasping_hand_)){
        ROS_INFO("valTask2::pickPanelTask performing shake test");
        task2_utils_->shakeTest(panel_grasping_hand_);
    }
    else
    {
        ROS_INFO("valTask2::pickPanelTask failed shake test. bag not in hand");
        eventQueue.riseEvent("/RE_REDETECT_PANEL");
    }

    if (task2_utils_->isPanelPicked(panel_grasping_hand_)){
        ROS_INFO("valTask2::pickPanelTask : Walking 1 step back");
        task2_utils_->movePanelToWalkSafePose(panel_grasping_hand_);
        ros::Duration(1).sleep();
        std::vector<float> x_offset={-0.3,-0.3};
        std::vector<float> y_offset={0.0,0.1};
        walker_->walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);
        ros::Duration(4).sleep();
        task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);
    }
    else
    {
        ROS_INFO("valTask2::pickPanelTask failed walking one step back. bag not in hand");
        eventQueue.riseEvent("/RE_REDETECT_PANEL");
    }

    if (task2_utils_->isPanelPicked(panel_grasping_hand_)){
        // walk slowly for this turn
        walker_->setWalkParms(1.0, 1.0, 0);
        /// @todo The following code should be a new state
        // reorient the robot.
        ROS_INFO("valTask2::pickPanelTask : Reorienting");
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
        walker_->walkToGoal(pose2D);
        ros::Duration(1).sleep();
        walker_->setWalkParms(0.7, 0.7, 0);
        task2_utils_->movePanelToWalkSafePose(panel_grasping_hand_);
        head_controller_->moveHead(0,0,0);
        ros::Duration(2).sleep();
    }
    else
    {
        ROS_INFO("valTask2::pickPanelTask failed taking a turn back. bag not in hand");
        eventQueue.riseEvent("/RE_REDETECT_PANEL");
    }

    if (task2_utils_->isPanelPicked(panel_grasping_hand_)){
        ROS_INFO("valTask2::pickPanelTask Picked pannel successfully !!!!!");
        eventQueue.riseEvent("/PICKED_PANEL");
    }
    else
    {
        ROS_INFO("valTask2::pickPanelTask failed dissapointingly fell somehow. bag not in hand");
        eventQueue.riseEvent("/RE_REDETECT_PANEL");
    }

    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::pickPanelTask waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectSolarArrayTask : executing " << name);

    if(solar_array_detector_ == nullptr) {
        solar_array_detector_ = new SolarArrayDetector(nh_, rover_walk_goal_waypoints_.back(), is_rover_on_right_);
        ros::Duration(0.2).sleep();
    }

    static int fail_count = 0;
    static int retry_count = 0;

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
        ROS_INFO("valTask2::detectSolarArrayTask : yaw:%f",pose2D.theta);
        retry_count = 0;

        if(solar_array_detector_ != nullptr) delete solar_array_detector_;
        solar_array_detector_ = nullptr;

        eventQueue.riseEvent("/DETECTED_ARRAY");

    }

    else if(retry_count < 5) {
        ROS_INFO("valTask2::detectSolarArrayTask : sleep 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_ARRAY_RETRY");
        ros::Duration(3).sleep();
    }
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        ROS_INFO("valTask2::detectSolarArrayTask : Failed 5 times. transitioning to error state");
        fail_count = 0;
        eventQueue.riseEvent("/DETECT_ARRAY_FAILED");
        if(solar_array_detector_ != nullptr) delete solar_array_detector_;
        solar_array_detector_ = nullptr;
    }
    // if failed retry detecting the panel
    else
    {
        ROS_INFO("valTask2::detectSolarArrayTask : Failed attempt. trying again");
        // increment the fail count
        fail_count++;
        eventQueue.riseEvent("/DETECT_ARRAY_RETRY");
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();


}

decision_making::TaskResult valTask2::walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::walkSolarArrayTask : executing " << name);
    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, solar_array_walk_goal_) ) {
        ROS_INFO("valTask2::walkSolarArrayTask : reached solar array");
        ros::Duration(1).sleep();
        /// @todo: check if robot rechead the panel (is this done ???)
        eventQueue.riseEvent("/REACHED_ARRAY");
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, solar_array_walk_goal_)) {
        ROS_INFO_STREAM("valTask2::walkSolarArrayTask : pose chaned to "<<solar_array_walk_goal_);
        walker_->walkToGoal(solar_array_walk_goal_, false);
        task2_utils_->resumePointCloud();
        // sleep so that the walk starts
        ROS_INFO("valTask2::walkSolarArrayTask : Footsteps should be generated now");
        ros::Duration(4).sleep();
        // update the previous pose
        pose_prev = solar_array_walk_goal_;
        eventQueue.riseEvent("/WALK_TO_ARRAY_EXECUTING");
    }
    // if walking stay in the same state
    else if (walk_track_->isWalking())
    {
        // no state change
        ROS_INFO_THROTTLE(2, "valTask2::walkSolarArrayTask : walking");
        eventQueue.riseEvent("/WALK_TO_ARRAY_EXECUTING");
    }
    // if walk finished
    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        ROS_INFO("valTask2::walkSolarArrayTask : walk failed");
        eventQueue.riseEvent("/WALK_TO_ARRAY_FAILED");
    }
    // if failed retry detecting the array and then walk
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("valTask2::walkSolarArrayTask : walk retry");
        eventQueue.riseEvent("/WALK_TO_ARRAY_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::walkSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::rotatePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::rotatePanelTask : executing " << name);
    if (!is_rotation_required_){
        ROS_INFO("valTask2::rotatePanelTask : Rotation is not required. skipping this state");
    }
    else {
        ROS_INFO("valTask2::rotatePanelTask : Rotating the panel");
        task2_utils_->rotatePanel(panel_grasping_hand_);
    }
    eventQueue.riseEvent("/ROTATED_PANEL");

    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::findCableIntermediateTask(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
    ROS_INFO_STREAM("valTask2::findCableIntermediateTask : executing " << name);

    static std::queue<float> head_yaw_ranges;

    static bool executingOnce = true;
    static float q1, q3;
    if(executingOnce)
    {
        // moving hands to get in position to detect the cable
        ROS_INFO("valTask2::findCableIntermediateTask : Moving panel to see cable");
        q3 = panel_grasping_hand_ == armSide::LEFT ? 0.5 : -0.5;
        arm_controller_->moveArmJoint((armSide)!panel_grasping_hand_, 3, q3);
        ros::Duration(0.5).sleep();
        q1 = panel_grasping_hand_ == armSide::LEFT ? -0.36 : 0.36;
        arm_controller_->moveArmJoint(panel_grasping_hand_,1,q1);
        ros::Duration(0.5).sleep();
        head_controller_->moveHead(0,30,0);
        ros::Duration(5).sleep();

        // possible head yaw rotations to better detect the cable
        //clear the queue before starting
        std::queue<float> temp;
        head_yaw_ranges= temp;
        head_yaw_ranges.push(-20.0);
        head_yaw_ranges.push(-10.0);
        head_yaw_ranges.push( 10.0);
        head_yaw_ranges.push( 20.0);

        executingOnce = false;
    }

    if (cable_detector_ == nullptr) {
        cable_detector_ = new CableDetector(nh_);
    }

    geometry_msgs::Pose cable_pose;
    int retry = 0;
    while (!cable_detector_->findCable(cable_pose) && retry++ < 15);

    if(cable_pose.position.x == 0 && !head_yaw_ranges.empty())
    {
        // wrong point detected. move head and try again
        head_controller_->moveHead(0,30,head_yaw_ranges.front());
        ROS_INFO("valTask2::findCableIntermediateTask : Cable not found. Trying with %.6f angle",head_yaw_ranges.front());
        head_yaw_ranges.pop();
        ros::Duration(5).sleep();

        eventQueue.riseEvent("/FIND_CABLE_RETRY");
    }
    else if (cable_pose.position.x == 0)
    {
        // state failed after detecting 5*4 times
        ROS_INFO("valTask2::findCableIntermediateTask : Cable not found. Failed after 20 trials");
        eventQueue.riseEvent("/FIND_CABLE_FAILED");

        executingOnce= true;
        if(cable_detector_ != nullptr) delete cable_detector_;
        cable_detector_ = nullptr;
    }
    else
    {
        // cable found
        setTempCablePose(cable_pose);
        std::cout<<"Cable position x: "<<cable_pose_temp_.position.x<<"\t"<<"Cable position y: "<<cable_pose_temp_.position.y<<"\t"<<"Cable position z: "<<cable_pose_temp_.position.z<<"\n";
        ROS_INFO("valTask2::findCableIntermediateTask : Cable found! Setting motion back. exiting");
        // set the poses back
        q3 = panel_grasping_hand_ == armSide::LEFT ? -1.85 : 1.85;
        q1 = panel_grasping_hand_ == armSide::LEFT ? -1.04 : 1.04;
        arm_controller_->moveArmJoint(panel_grasping_hand_,1,q1);
        ros::Duration(0.2).sleep();
        arm_controller_->moveArmJoint((armSide)!panel_grasping_hand_, 3, q3);
        eventQueue.riseEvent("/FOUND_CABLE");
        executingOnce= true;
        if(cable_detector_ != nullptr) delete cable_detector_;
        cable_detector_ = nullptr;
    }


    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectSolarArrayFineTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectSolarArrayFineTask : executing " << name);

    static int retry_count = 0;
    if(solar_array_fine_detector_ == nullptr) {
        solar_array_fine_detector_ = new ArrayTableDetector(nh_, cable_pose_temp_.position);
    }

    // detect solar array
    std::vector<geometry_msgs::Pose> poses;
    solar_array_fine_detector_->getDetections(poses);

    // if we get atleast two detections
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
        ROS_INFO("valTask2::detectSolarArrayFineTask : Position x:%f y:%f z:%f",poses[idx].position.x,poses[idx].position.y,poses[idx].position.z);
        ROS_INFO("valTask2::detectSolarArrayFineTask : yaw:%f",pose2D.theta);
        retry_count = 0;

        eventQueue.riseEvent("/DETECTED_ARRAY_FINE");

        if(solar_array_fine_detector_ != nullptr) delete solar_array_fine_detector_;
        solar_array_fine_detector_ = nullptr;

        pelvis_controller_->controlPelvisHeight(1.1);
        ros::Duration(2).sleep();
    }

    else if(retry_count < 5) {
        ROS_INFO("valTask2::detectSolarArrayFineTask : sleep 3 seconds for panel detection");
        ++retry_count;
        eventQueue.riseEvent("/DETECT_ARRAY_FINE_RETRY");
        ros::Duration(3).sleep();
    }
    // if failed for more than 5 times, go to error state
    else
    {
        ROS_INFO("valTask2::detectSolarArrayFineTask : Failed 5 times. transitioning to error state");
        retry_count = 0;
        eventQueue.riseEvent("/DETECT_ARRAY_FINE_FAILED");
        if(solar_array_fine_detector_ != nullptr) delete solar_array_fine_detector_;
        solar_array_fine_detector_ = nullptr;
    }

    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::detectSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();

}



decision_making::TaskResult valTask2::alignSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::alignSolarArrayTask : executing " << name);
    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);
    task2_utils_->clearBoxPointCloud(CLEAR_BOX_CLOUD::FULL_BOX);
    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, solar_array_fine_walk_goal_) ) {
        ROS_INFO("valTask2::alignSolarArrayTask : reached solar array");
        ros::Duration(1).sleep();
        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/ALIGNED_TO_ARRAY");
    }
    // check if the pose is changed
    else if (taskCommonUtils::isPoseChanged(pose_prev, solar_array_fine_walk_goal_)) {
        ROS_INFO_STREAM("valTask2::alignSolarArrayTask : pose chaned to "<<solar_array_fine_walk_goal_);
        walker_->walkToGoal(solar_array_fine_walk_goal_, false);
        // sleep so that the walk starts
        ROS_INFO("valTask2::alignSolarArrayTask : Footsteps should be generated now");
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
        ROS_INFO("valTask2::alignSolarArrayTask : walk failed");
        eventQueue.riseEvent("/ALIGN_TO_ARRAY_FAILED");
    }
    // if failed retry detecting the array and then walk
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("valTask2::alignSolarArrayTask : walk retry");
        eventQueue.riseEvent("/ALIGN_TO_ARRAY_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::alignSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::placePanelTask : executing " << name);
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

        arm_controller_->moveToZeroPose(panel_grasping_hand_);
        ros::Duration(0.5).sleep();
        arm_controller_->moveToZeroPose((armSide)!panel_grasping_hand_);
        ros::Duration(0.5).sleep();
        handsPulledOff = true;
        eventQueue.riseEvent("/PLACE_ON_GROUND_RETRY");
    }
    else if (task2_utils_->getCurrentCheckpoint() > 2){
        ROS_INFO("valTask2::placePanelTask : Placed the panel successfully");
        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(1).sleep();
        eventQueue.riseEvent("/PLACED_ON_GROUND");
    }
    else {
        ROS_INFO("valTask2::placePanelTask : Panel placement failed");
        eventQueue.riseEvent("/PLACED_ON_GROUND_FAILED");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::deployPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    if(button_detector_ == nullptr) {
        button_detector_ = new ButtonDetector(nh_);
    }

    static int retry_count = 0;

    if (retry_count == 0){
        //tilt head downwards to see the panel
        head_controller_->moveHead(0.0f, 40.0f, 0.0f);
        //wait for head to be in position
        ros::Duration(2).sleep();
    }

    //detect button
    if( button_detector_->findButtons(button_coordinates_)){

        ROS_INFO_STREAM("Button detected at "<<button_coordinates_.x<< " , "<<button_coordinates_.y<<" , "<<button_coordinates_.z);

        // generate the event
        eventQueue.riseEvent("/BUTTON_DETECTED");

        if (button_detector_ != nullptr) delete button_detector_;
        button_detector_ = nullptr;
    }
    else if( retry_count++ < 10){
        ROS_INFO("Did not detect button, retrying");
        eventQueue.riseEvent("/DETECT_BUTTON_RETRY");
    }
    else{
        ROS_INFO("Did not detect button, failed");
        eventQueue.riseEvent("/BUTTON_DETECTION_FAILED");
        retry_count = 0;
        if (button_detector_ != nullptr) delete button_detector_;
        button_detector_ = nullptr;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::alignSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    if(button_press_ == nullptr) {
        button_press_ = new ButtonPress(nh_);
    }

    static std::queue<geometry_msgs::Point> points;
    static bool executeOnce =true;
    if(executeOnce)
    {
        ROS_INFO("valTask2::deployPanelTask : setting up possible button locations for retry");
        //clear the queue before starting
        std::queue<geometry_msgs::Point> temp;
        points= temp;

        points.push(button_coordinates_);

        // define 4 retry points before going back to detection
        float offset=0.02;
        geometry_msgs::Point buttonPelvis;
        robot_state_->transformPoint(button_coordinates_,buttonPelvis,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
        std::vector<geometry_msgs::Point> retryPoints;

        retryPoints.resize(4);
        // First Retry Point
        retryPoints[0]=buttonPelvis;
        retryPoints[0].x+=offset;
        robot_state_->transformPoint(retryPoints[0],retryPoints[0],VAL_COMMON_NAMES::PELVIS_TF);

        // Second Retry Point
        retryPoints[1]=buttonPelvis;
        retryPoints[1].x-=offset;
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

    if(task2_utils_->getCurrentCheckpoint() == 3)
    {
        ROS_INFO("valTask2::deployPanelTask : Panel deployed successfully");
        eventQueue.riseEvent("/DEPLOYED");

        if (button_press_ != nullptr) delete button_press_;
        button_press_ = nullptr;
    }
    else if(!points.empty())
    {
        ROS_INFO("valTask2::deployPanelTask : Reaching out to the button");
        // choosing default side to be left at the moment. it might work with right hand as well.
        button_press_->pressButton(LEFT,points.front());
        points.pop();
        executeOnce=false;
        eventQueue.riseEvent("/DEPLOY_RETRY");
    }
    else if(points.empty() && retry_count <5)
    {
        ROS_INFO("valTask2::deployPanelTask : deployment failed for all 5 points. retrying");
        retry_count++;
        executeOnce=true;
        eventQueue.riseEvent("/DETECT_AGAIN");
        if (button_press_ != nullptr) delete button_press_;
        button_press_ = nullptr;
    }
    else if(retry_count >5)
    {
        ROS_INFO("valTask2::deployPanelTask : Failed 20 times. time to exit");
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
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectCableTask : executing " << name);

    if(cable_detector_ == nullptr) {
        cable_detector_ = new CableDetector(nh_);
    }

    static int retry_count = 0;

    if (retry_count == 0 ){
        //tilt head downwards to see the panel
        head_controller_->moveHead(0.0f, 40.0f, 0.0f, 2.0f);
        //wait for head to be in position
        ros::Duration(3).sleep();
    }

    //detect cable
    if( cable_detector_->findCable(cable_pose_)){

        ROS_INFO_STREAM("valTask2::detectCableTask : Cable detected at "<<cable_pose_.position.x<< " , "<<cable_pose_.position.y<<" , "<<cable_pose_.position.z);

        retry_count = 0;
        // generate the event
        eventQueue.riseEvent("/DETECTED_CABLE");
    }
    else if( retry_count++ < 10){
        ROS_INFO("valTask2::detectCableTask : Did not detect cable, retrying");
        eventQueue.riseEvent("/DETECT_CABLE_RETRY");
    }
    else {
        ROS_INFO("valTask2::detectCableTask : Did not detect cable, failed");
        eventQueue.riseEvent("/DETECT_CABLE_FAILED");
        retry_count = 0;
        if (cable_detector_ != nullptr) delete cable_detector_;
        cable_detector_ = nullptr;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::deployPanelTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::pickCableTask : executing " << name);

    /*
     * isCableOnTable   -- pickup the cable and retry
     * isCableInHand    -- task complete continue
     * else - fail
     */
    static int retry = 0;

    if(cable_task_ == nullptr)
    {
        cable_task_ = new CableTask(nh_);
    }

    if (task2_utils_->isCableOnTable(cable_pose_)){

        ROS_INFO("valTask2::pickCableTask : Picking up the cable");
        cable_task_->grasp_choke(armSide::RIGHT,cable_pose_);
        ros::Duration(1).sleep();
    }

    if(task2_utils_->getCurrentCheckpoint() == 4 && task2_utils_->isCableInHand(armSide::RIGHT))    //always use right hand for cable pickup
    {
        ROS_INFO("valTask2::pickCableTask : Picked up cable");
        // go to next state
        eventQueue.riseEvent("/CABLE_PICKED");

    }
    else if(retry <5)
    {
        ROS_INFO("valTask2::pickCableTask : Failed picking up the cable. Retrying detection");
        retry++;
        // drop cable motion here
        eventQueue.riseEvent("/PICKUPCABLE_RETRY");
    }
    else
    {
        ROS_INFO("valTask2::pickCableTask : Failed picking the cable.");
        if (cable_detector_ != nullptr) delete cable_detector_;
        cable_detector_ = nullptr;
        eventQueue.riseEvent("/PICKUP_CABLE_FAILED");
    }

    // generate the event
    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::pickCableTask : waiting for transition");
    }

    return TaskResult::SUCCESS();
}

TaskResult valTask2::detectSocketTask(string name, const FSMCallContext &context, EventQueue &eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectSocketTask : executing " << name);
    if (socket_detector_ == nullptr){
        socket_detector_ = new plug_detector(nh_);
    }

    static int retry_count = 0;

    if (retry_count == 0){
        //tilt head downwards to see the panel
        head_controller_->moveHead(0.0f, 40.0f, 0.0f);
        //wait for head to be in position
        ros::Duration(2).sleep();
    }

    //detect socket
    if( socket_detector_->findPlug(socket_coordinates_)){

        ROS_INFO_STREAM("valTask2::detectSocketTask : Socket detected at "<<socket_coordinates_.x<< " , "<<socket_coordinates_.y<<" , "<<socket_coordinates_.z);

        // generate the event
        eventQueue.riseEvent("/DETECTED_SOCKET");

        if (socket_detector_ != nullptr) delete socket_detector_;
        socket_detector_ = nullptr;
    }
    else if( retry_count++ < 10){
        ROS_INFO("valTask2::detectSocketTask : Did not detect button, retrying");
        eventQueue.riseEvent("/DETECT_SOCKET_RETRY");
    }
    else{
        ROS_INFO("valTask2::detectSocketTask : Did not detect button, failed");
        eventQueue.riseEvent("/DETECT_SOCKET_FAILED");
        retry_count = 0;
        if (socket_detector_ != nullptr) delete socket_detector_;
        socket_detector_ = nullptr;
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("valTask2::alignSolarArrayTask : waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    if (cable_task_ != nullptr) delete cable_task_;
    cable_task_ = nullptr;

    static int retry=0;
    static int fail =0;

    //    Checkpoint 5: Plug the power cable into the solar panel
    if(cable_task_->insert_cable(socket_coordinates_) && task2_utils_->getCurrentCheckpoint() == 5)
    {
        ROS_INFO("valTask2::plugCableTask plugged the cable succesfully. going to next state");
        eventQueue.riseEvent("/CABLE_PLUGGED");
    }
    else if(task2_utils_->isCableInHand(armSide::RIGHT) && retry <5)
    {
        ROS_INFO("valTask2::plugCableTask cable is in hand. attempting to retry plugging the cable");
        retry++;
        eventQueue.riseEvent("/PLUGIN_CABLE_RETRY");
    }
    else if(fail <5)
    {
        ///@todo function to drop cable at seed point
        ROS_INFO("valTask2::plugCableTask retried 5 times. going back to detect the cable");
        fail++;
        retry=0;
        eventQueue.riseEvent("/REDETECT_CABLE");
    }
    else
    {
        ROS_INFO("valTask2::plugCableTask failed all attempts. going to error state");
        eventQueue.riseEvent("/PLUGIN_CABLE_FAILED");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask2::detectFinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask2::detectFinishBoxTask :executing " << name);

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
        ros::Duration(3).sleep();   // this will execute only when visited map is not updated which should never happen
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    } else  if(finish_box_detector_->getFinishBoxCenters(detections)){
        for(size_t i = 0; i < detections.size(); ++i){
            size_t index = MapGenerator::getIndex(detections[i].x, detections[i].y);
            ROS_INFO("Index in map %d and size of visited map is %d", (int)index, (int)visited_map_.data.size());
            if(visited_map_.data.at(index) == CELL_STATUS::VISITED){
                continue;
            }

            next_finishbox_center_.x = detections[i].x;
            next_finishbox_center_.y = detections[i].y;
            next_finishbox_center_.theta = atan2(next_finishbox_center_.y, next_finishbox_center_.x);
            ROS_INFO("detectFinishBoxTask: Successful");
            eventQueue.riseEvent("/DETECT_FINISH_SUCESSFUL");
            if(finish_box_detector_ != nullptr) delete finish_box_detector_;
            finish_box_detector_ = nullptr;
            //sleep is required to avoid moving to next state before subscriber is shutdown
            //            ros::Duration(2).sleep();
            break;
        }
        // this is to avoid detecting points that will always be in collision
        retry_count++;
    }
    else if(retry_count++ < 5){
        ros::Duration(3).sleep();   // map is updated every 4 sec
        eventQueue.riseEvent("/DETECT_FINISH_RETRY");
    }
    else{
        eventQueue.riseEvent("/DETECT_FINISH_FAILED");
        if(finish_box_detector_ != nullptr) delete finish_box_detector_;
        finish_box_detector_ = nullptr;
        //sleep is required to avoid moving to next state before subscriber is shutdown
        //        ros::Duration(2).sleep();
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("detectfinishBoxTask: waiting for transition");
    }

    return TaskResult::SUCCESS();

}

decision_making::TaskResult valTask2::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {

    ROS_INFO_STREAM("valTask2::walkToFinishTask : executing " << name);

    static bool execute_once = true;

    if (execute_once){
        // set the robot to default state to walk
        gripper_controller_->openGripper(armSide::RIGHT);
        ros::Duration(0.2).sleep();
        gripper_controller_->openGripper(armSide::LEFT);
        ros::Duration(1).sleep();
        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(0.2).sleep();
        chest_controller_->controlChest(0.0, 0.0, 0.0);
        ros::Duration(0.2).sleep();
        arm_controller_->moveToDefaultPose(armSide::LEFT);
        ros::Duration(0.2).sleep();
        arm_controller_->moveToDefaultPose(armSide::RIGHT);
        ros::Duration(0.2).sleep();
        execute_once = false;
    }

    static int fail_count = 0;

    // walk to the goal location

    static geometry_msgs::Pose2D pose_prev;

    geometry_msgs::Pose current_pelvis_pose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);

    // check if the pose is changed
    if ( taskCommonUtils::isGoalReached(current_pelvis_pose, next_finishbox_center_)) {
        ROS_INFO("walkToFinishTask: reached panel");
        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/WALK_TO_FINISH_SUCESSFUL");
    }
    // the goal can be updated on the run time
    else if (taskCommonUtils::isPoseChanged(pose_prev, next_finishbox_center_))
    {
        ROS_INFO("walkToFinishTask: pose changed");
        walker_->walkToGoal(next_finishbox_center_, false);
        ROS_INFO("walkToFinishTask: Footsteps should be published now");
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
        eventQueue.riseEvent("/WALK_TO_FINISH_ERROR");
    }
    // if failed retry detecting the panel and then walk
    // also handles MOVE_FAILED
    else
    {
        // increment the fail count
        fail_count++;
        ROS_INFO("walkToFinishTask: walk retry");
        eventQueue.riseEvent("/WALK_TO_FINISH_RETRY");
    }

    // wait infinetly until an external even occurs
    while(!preemptiveWait(1000, eventQueue)){
        ROS_INFO("walkToFinishTask: waiting for transition");
    }

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

void valTask2::setTempCablePose(geometry_msgs::Pose value)
{
    cable_pose_temp_ = value;
}
