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
    handle_detector_    = new handle_detector(nh_);

    // controllers
    chest_controller_   = new chestTrajectory(nh_);
    pelvis_controller_  = new pelvisTrajectory(nh_);
    head_controller_    = new HeadTrajectory(nh_);

    map_update_count_ = 0;
    occupancy_grid_sub_ = nh_.subscribe("/map",10, &valTask1::occupancy_grid_cb, this);

    // TODO: just for vis remove later
    array_pub_    = nh.advertise<visualization_msgs::MarkerArray>( "Circle_Array", 0 );
}

// destructor
valTask1::~valTask1(){

    delete walker_;
    delete walk_track_;
    delete panel_detector_;

}

void valTask1::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){
    ++map_update_count_;
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
    // the state transition can happen from an event externally or can be geenerated here
    //!!!!! depends on the developer and use case
    ROS_INFO("Occupancy Grid has been updated %d times, tried %d times", map_update_count_, retry_count);

    if (map_update_count_ < 2 && retry_count++ < 10) {
        ROS_INFO("Wait for occupancy grid to be updated with atleast 2 messages");
        ros::Duration(4.0).sleep();
        eventQueue.riseEvent("/INIT_RETRY");
    }
    else if(retry_count > 9){
        ROS_INFO("Failed to initialize");
        eventQueue.riseEvent("/INIT_FAILED");
    }
    else{
        // move to a configuration that is robust while walking
        pelvis_controller_->controlPelvisHeight(0.9);
        chest_controller_->controlChest(0.0f, 19.0f, 0.0f);
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
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    if(panel_detector_ == nullptr) {
        panel_detector_ = new panel_detector(nh_);
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
        setPanelWalkGoal(pose2D);

        std::cout << "quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w <<std::endl;
        std::cout << "yaw: " << pose2D.theta  <<std::endl;

        // update the plane coeffecients
        setPanelCoeff(panel_detector_->getPanelPlaneModel());

        eventQueue.riseEvent("/DETECTED_PANEL");
        delete panel_detector_;
        panel_detector_ = nullptr;
    }

    else if(retry_count < 5) {
        ROS_INFO("sleep for 3 seconds for panel detection");
        ++retry_count;
        ros::Duration(3).sleep();
        eventQueue.riseEvent("/DETECT_PANEL_RETRY");
    }

    // if failed for more than 5 times, go to error state
    else if (fail_count > 5)
    {
        // reset the fail count
        fail_count = 0;
        eventQueue.riseEvent("DETECT_PANEL_FAILED");
        delete panel_detector_;
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


decision_making::TaskResult valTask1::walkToControlPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    static int fail_count = 0;

    // walk to the goal location
    // the goal can be updated on the run time
    static geometry_msgs::Pose2D pose_prev;

    // check if the pose is changed
    if (isPoseChanged(pose_prev, panel_walk_goal_))
    {
        ROS_INFO("pose chaned");
        //walker_->walkToGoal(panel_walk_goal_, false);
        // sleep so that the walk starts
        //ros::Duration(4).sleep();

        // update the previous pose
        pose_prev = panel_walk_goal_;
    }

    // if walking stay in the same state
    if (walk_track_->isWalking())
    {
        // no state change
        ROS_INFO("walking");
        eventQueue.riseEvent("/WALK_EXECUTING");
    }
    // if walk finished
    else if (!walk_track_->isWalking())
    {
        ROS_INFO("reached panel");

        // TODO: check if robot rechead the panel
        eventQueue.riseEvent("/REACHED_PANEL");
    }
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
    head_controller_->moveHead(0.0f, 30.0f, 0.0f, 2.0f);

    //wait for head to be in position
    ros::Duration(3).sleep();

    //detect handles
    if( handle_detector_->findHandles(handle_loc_)){

        ROS_INFO_STREAM("Handles detected at "<<handle_loc_[0]<< " : "<<handle_loc_[1]);

        // walk 0.4m forward
        // walker_->walkNSteps(1, 0.0, 0.4, false);

        // generate the event
        eventQueue.riseEvent("/DETECTED_HANDLE");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::adjustArmTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the way points to move the handle
    std::vector<geometry_msgs::Pose> waypoints;
    RobotStateInformer* transformer = RobotStateInformer::getRobotStateInformer(nh_);
    geometry_msgs::Pose point;
    point.position.x = handle_loc_[0].x;
    point.position.y = handle_loc_[0].y;
    point.position.z = handle_loc_[0].z;
    transformer->getCurrentPose(VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, point);

    handle_loc_[0].x = point.position.x;
    handle_loc_[0].y = point.position.y;
    handle_loc_[0].z = point.position.z;
    createHandleWayPoints(handle_loc_[0], waypoints);

    // generate the event
    while(!preemptiveWait(1000, eventQueue)){
    eventQueue.riseEvent("/ADJUST_ARMS_RETRY");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/YAW_CORRECTION_SUCESSFUL");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectfinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    eventQueue.riseEvent("/WALK_TO_END");
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
    return panel_walk_goal_;
}

void valTask1::setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal)
{
    panel_walk_goal_ = panel_walk_goal;
}

void valTask1::setPanelCoeff(const std::vector<float> &panel_coeff)
{
  panel_coeff_ = panel_coeff;
}

bool valTask1::isPoseChanged(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new)
{
    bool ret = false;

//    ROS_INFO("%f", pose_new.x);
//    ROS_INFO("%f", pose_new.y);
//    ROS_INFO("%f", pose_old.x);
//    ROS_INFO("%f", pose_old.y);
//    ROS_INFO("%f", sqrt(pow((pose_new.y - pose_old.y),2) + pow((pose_new.x - pose_old.x),2)));

    if (sqrt(pow((pose_new.y - pose_old.y),2) + pow((pose_new.x - pose_old.x),2)) > 0.02) // > 2cm
    {
        ret = true;
    }
    else if (fabs(fmod(pose_new.theta,(2*M_PI)) - fmod(pose_new.theta,(2*M_PI))) > 0.0174533) //1 degree
    {
        ret = true;
    }

    return ret;
}

// !!!! make sure this is called after panel is detected and handels are detected
void valTask1::createHandleWayPoints(const geometry_msgs::Point &center, std::vector<geometry_msgs::Pose> &points)
{
  float radius = 0.13;
  int num_steps = 20;

  // clear the points
  points.clear();

  ROS_INFO_STREAM("loc1 "<<handle_loc_[1] << " loc0 " <<handle_loc_[0]);
  ROS_INFO_STREAM("center "<< center);
  ROS_INFO("radius %f",sqrt(pow(handle_loc_[1].x - handle_loc_[0].x, 2) + pow(handle_loc_[1].y - handle_loc_[0].y,2)));
  //RobotStateInformer* transformer = RobotStateInformer::getRobotStateInformer(nh_);

  for (int i=0; i<num_steps; i++)
  {
    // circle parametric equation
    //geometry_msgs::PoseStamped point;
    geometry_msgs::Pose point;
    point.position.x = center.x + (radius * cos((float)i*(2*M_PI/num_steps)));
    point.position.y = center.y + (radius * sin((float)i*(2*M_PI/num_steps)));
    // get the z from the plane equation
    // z = -ax - by - d/c
    point.position.z = ((-panel_coeff_[0] * point.position.x) - (panel_coeff_[1] * point.position.y) - panel_coeff_[3])/panel_coeff_[2];
    //transformer->getCurrentPose(VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, point);
    points.push_back(point);
  }

  // TODO: should be rmoved
  // visulation of the circle
  visualization_msgs::MarkerArray circle = visualization_msgs::MarkerArray();
  for (int i = 0; i < num_steps; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "circle";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = points[i].position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    circle.markers.push_back(marker);
  }

  array_pub_.publish( circle );
}
