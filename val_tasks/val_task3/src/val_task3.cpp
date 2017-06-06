#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <val_task3/val_task3.h>
#include <srcsim/StartTask.h>
#include <queue>

#define foreach BOOST_FOREACH

valTask3* valTask3::currentObject = nullptr;

valTask3* valTask3::getValTask3(ros::NodeHandle nh){

  if  (currentObject == nullptr){
    currentObject = new valTask3(nh);
    return currentObject;
  }

  ROS_ERROR("Object already exists");
  assert(false && "Object already exists");
}


valTask3::valTask3(ros::NodeHandle nh):nh_(nh){

  // walker
  walker_              = new ValkyrieWalker(nh_,0.7,0.7,0,0.18);
  ROS_INFO("1");
  // walk tracker
  walk_track_          = new walkTracking(nh_);
  ROS_INFO("2");

  // controllers
  chest_controller_    = new chestTrajectory(nh_);
  pelvis_controller_   = new pelvisTrajectory(nh_);
  head_controller_     = new HeadTrajectory(nh_);
  gripper_controller_  = new gripperControl(nh_);
  arm_controller_      = new armTrajectory(nh_);
  wholebody_controller_= new wholebodyManipulation(nh_);
  task3_utils_         = new task3Utils(nh_);
  ROS_INFO("4");

  // detectors


  robot_state_         = RobotStateInformer::getRobotStateInformer(nh_);
  map_update_count_    = 0;
  occupancy_grid_sub_  = nh_.subscribe("/map",10,&valTask3::occupancy_grid_cb,this);
}

valTask3::~valTask3(){

  // shut down subscribers
  occupancy_grid_sub_.shutdown();

  if(walker_ != nullptr)                delete walker_;
  if(walk_track_ != nullptr)            delete walk_track_;
  if(chest_controller_ != nullptr)      delete chest_controller_;
  if(pelvis_controller_ != nullptr)     delete pelvis_controller_;
  if(head_controller_ != nullptr)       delete head_controller_;
  if(gripper_controller_ != nullptr)    delete gripper_controller_;
  if(arm_controller_ != nullptr)        delete arm_controller_;
  if(wholebody_controller_ != nullptr)  delete wholebody_controller_;
  if(task3_utils_ != nullptr)           delete task3_utils_;
}

void valTask3::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){
    ++map_update_count_;
}


bool valTask3::preemptiveWait(double ms, decision_making::EventQueue &queue){

  for(size_t i = 0; i<100 && !queue.isTerminated(); ++i){
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms/100.0));
  }

  return queue.isTerminated();
}


decision_making::TaskResult valTask3::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("valTask3::initTask : executing " << name);
    static int retry_count = 0;

    // reset map count for the first entry
    if(retry_count == 0){
       map_update_count_ = 0;
    }

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
        srv.request.task_id       = 3;
        if(client.call(srv)) {
            //what do we do if this call fails or succeeds?
        }
        // generate the event
        eventQueue.riseEvent("/INIT_SUCESSFUL");

    }
    else if (map_update_count_ < 2 && retry_count++ < 40) {
        ROS_INFO("valTask3::initTask : Retry Count : %d. Wait for occupancy grid to be updated with atleast 2 messages", retry_count);
        ros::Duration(2.0).sleep();
        eventQueue.riseEvent("/INIT_RETRY");
    }
    else {
        retry_count = 0;
        ROS_INFO("valTask3::initTask : Failed to initialize");
        eventQueue.riseEvent("/INIT_FAILED");
    }
    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}


decision_making::TaskResult valTask3::detectStepsTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}


decision_making::TaskResult valTask3::climbStepsTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectDoorHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::openDoorTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkthroughDoorTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectLeakToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToLeakToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::pickLeakTool(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToDetectLeak(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectLeakTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectRepairToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToRepairToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::pickRepairTool(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToLeakTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::leakRepairTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

    ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

// helper functions
void valTask3::setRepairToolLoc(const geometry_msgs::Point &repair_tool_loc)
{
  repair_tool_loc_ = repair_tool_loc;
}

void valTask3::setLeakLoc(const geometry_msgs::Point &leak_loc)
{
  leak_loc_ = leak_loc;
}

void valTask3::setLeakWallPose(const geometry_msgs::Pose2D &leak_wall_pose)
{
  leak_wall_pose_ = leak_wall_pose;
}

void valTask3::setLeakDetectorLoc(const geometry_msgs::Point &leak_detector_loc)
{
  leak_detector_loc_ = leak_detector_loc;
}

void valTask3::setTableWalkPose(const geometry_msgs::Pose2D &table_walk_pose)
{
  table_walk_pose_ = table_walk_pose;
}

void valTask3::setHandleCenter(const geometry_msgs::Point &handle_center)
{
  handle_center_ = handle_center;
}

void valTask3::setStairDetectWalkPose(const geometry_msgs::Pose2D &stair_detect_walk_pose)
{
  stair_detect_walk_pose_ = stair_detect_walk_pose;
}

void valTask3::setFinishBoxPose(const geometry_msgs::Pose2D &finish_box_pose)
{
  finish_box_pose_ = finish_box_pose;
}

void valTask3::resetRobotToDefaults(int arm_pose)
{
    // open grippers
    gripper_controller_->openGripper(armSide::RIGHT);
    ros::Duration(0.2).sleep();
    gripper_controller_->openGripper(armSide::LEFT);
    ros::Duration(0.2).sleep();

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
        ros::Duration(0.2).sleep();
        arm_controller_->moveToZeroPose(armSide::RIGHT);
        ros::Duration(1).sleep();
    }
    else if (arm_pose == 1)
    {
        arm_controller_->moveToDefaultPose(armSide::LEFT);
        ros::Duration(0.2).sleep();
        arm_controller_->moveToDefaultPose(armSide::RIGHT);
        ros::Duration(1).sleep();
    }

    // neck to defaults
    head_controller_->moveHead(0.0f, 0.0f, 0.0f, 0.0f);
}
