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
#include <val_task1/val_task1.h>

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
    panel_detector_ = new panel_detector(nh);
}

// destructor
valTask1::~valTask1(){

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

    // the state transition can happen from an event externally or can be geenerated here
    //!!!!! depends on the developer and use case

    // generate the event
    eventQueue.riseEvent("/INIT_SUCESSFUL");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

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
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(tf::Quaternion(poses[idx].orientation.x, poses[idx].orientation.y, poses[idx].orientation.z, poses[idx].orientation.w));
        //mat.setValue(poses[idx].orientation.x, poses[idx].orientation.y, poses[idx].orientation.z, poses[idx].orientation.w);
        mat.getEulerYPR(yaw, pitch, roll);
        pose2D.theta = yaw;
        setPanelWalkGoal(pose2D);

        std::cout << "quat " << poses[idx].orientation.x << " " <<poses[idx].orientation.y <<" "<<poses[idx].orientation.z <<" "<<poses[idx].orientation.w <<std::endl;
        std::cout << "yaw: " << yaw << " pitch " << pitch << " roll " << roll << std::endl;

        eventQueue.riseEvent("/DETECTED_PANEL");
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

    // check if the pose is started
    if (isPoseChanged(pose_prev, panel_walk_goal_))
    {
        ROS_INFO("pose chaned");
        walker_->walkToGoal(panel_walk_goal_, false);
        // sleep so that the walk starts
        sleep(10);

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

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask1::adjustArmTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    ROS_INFO_STREAM("executing " << name);

    // generate the event
    //eventQueue.riseEvent("/INIT_SUCESSUFL");

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


bool valTask1::isPoseChanged(geometry_msgs::Pose2D pose_old, geometry_msgs::Pose2D pose_new)
{
    bool ret = false;

    ROS_INFO("%f", pose_new.x);
    ROS_INFO("%f", pose_new.y);
    ROS_INFO("%f", pose_old.x);
    ROS_INFO("%f", pose_old.y);
    ROS_INFO("%f", sqrt(pow((pose_new.y - pose_old.y),2) + pow((pose_new.x - pose_old.x),2)));

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
