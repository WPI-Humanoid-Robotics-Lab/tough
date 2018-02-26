/*
 *
 *
 *
 * */

#include "tough_footstep/RobotWalker.h"
#include "tough_common/tough_common_names.h"
#include <iostream>
#include <ros/ros.h>

int RobotWalker::id = -1;

// constructor
RobotWalker::RobotWalker(ros::NodeHandle nh,double InTransferTime ,double InSwingTime, int InMode, double swingHeight):nh_(nh)
{
    std::string robot_name;
    nh_.getParam("/ihmc_ros/robot_name",robot_name);

    this->footsteps_pub_   = nh_.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/"+robot_name+"/control/footstep_list",1,true);
    this->footstep_status_ = nh_.subscribe("/ihmc_ros/"+robot_name+"/output/footstep_status", 20,&RobotWalker::footstepStatusCB, this);
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    this->nudgestep_pub_   = nh_.advertise<ihmc_msgs::FootTrajectoryRosMessage>("/ihmc_ros/"+robot_name+"/control/foot_trajectory",1,true);
    this->loadeff_pub      = nh_.advertise<ihmc_msgs::EndEffectorLoadBearingRosMessage>("/ihmc_ros/"+robot_name+"/control/end_effector_load_bearing",1,true);

    transfer_time_  = InTransferTime;
    swing_time_     = InSwingTime;
    execution_mode_ = InMode;
    swing_height_   = swingHeight;

    ros::Duration(0.5).sleep();
    step_counter_ = 0;

    right_foot_frame_.data = rd_->getRightFootFrameName();
    left_foot_frame_.data  = rd_->getLeftFootFrameName();

    //start timer
    cbTime_=ros::Time::now();

}

// Destructor
RobotWalker::~RobotWalker(){
}


// CallBack function for walking status
void RobotWalker::footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage &msg)
{
    if(msg.status == 1)
    {
        step_counter_++;
        //ROS_INFO("step counter : %d",step_counter);
    }

    // reset the timer
    cbTime_=ros::Time::now();

    return;
}

// calls the footstep planner to plan path and walks to a 2D goal.
bool RobotWalker::walkToGoal( geometry_msgs::Pose2D &goal, bool waitForSteps)
{    
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_duration = transfer_time_;
    list.default_swing_duration    = swing_time_;
    list.execution_mode = execution_mode_;
    list.unique_id = RobotWalker::id;

    if(this->getFootstep(goal,list))
    {
        this->footsteps_pub_.publish(list);
        RobotWalker::id--;

        if (waitForSteps)
        {
            cbTime_=ros::Time::now();
            this->waitForSteps(list.footstep_data_list.size());
        }
        return true;
    }
    return false;
}

// walks certain number of defined footsteps. steps defined wrt world frame.
bool RobotWalker::walkNSteps(int numSteps, float xOffset, float yOffset, bool continous, RobotSide startLeg, bool waitForSteps)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_duration = transfer_time_;
    list.default_swing_duration = swing_time_;
    list.execution_mode = execution_mode_;

    list.unique_id = RobotWalker::id ;

    for (int m =1; m <= numSteps ; m++) {
        if(m%2 == 1) {
            list.footstep_data_list.push_back(*getOffsetStep(startLeg , m*xOffset, m*yOffset));
        }
        else {
            list.footstep_data_list.push_back(*getOffsetStep((startLeg+1)%2 , m*xOffset, m*yOffset));
        }

    }
    if(!continous){
        if (numSteps%2 ==1) {
            list.footstep_data_list.push_back(*getOffsetStep((startLeg+1)%2  , numSteps*xOffset, numSteps*yOffset));
        }
        if (numSteps%2 ==0) {
            list.footstep_data_list.push_back(*getOffsetStep(startLeg , numSteps*xOffset, numSteps*yOffset));
        }
    }

    this->walkGivenSteps(list, waitForSteps);
    return true;
}

// walks certain number of defined footsteps. steps defined wrt pelvis frame.
bool RobotWalker::walkNStepsWRTPelvis(int numSteps, float xOffset, float yOffset, bool continous, RobotSide startLeg, bool waitForSteps)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_duration = transfer_time_;
    list.default_swing_duration = swing_time_;
    list.execution_mode = execution_mode_;

    list.unique_id = RobotWalker::id ;

    for (int m =1; m <= numSteps ; m++) {
        if(m%2 == 1) {
            list.footstep_data_list.push_back(*getOffsetStepWRTPelvis(startLeg , m*xOffset, m*yOffset));
        }
        else {
            list.footstep_data_list.push_back(*getOffsetStepWRTPelvis((startLeg+1)%2 , m*xOffset, m*yOffset));
        }

    }
    if(!continous){
        if (numSteps%2 ==1) {
            list.footstep_data_list.push_back(*getOffsetStepWRTPelvis((startLeg+1)%2  , numSteps*xOffset, numSteps*yOffset));
        }
        if (numSteps%2 ==0) {
            list.footstep_data_list.push_back(*getOffsetStepWRTPelvis(startLeg , numSteps*xOffset, numSteps*yOffset));
        }
    }

    this->walkGivenSteps(list, waitForSteps);
    return true;
}

// walks predefined steps which could have varying step length and step widths. This is defined wrt World frame.
bool RobotWalker::walkPreComputedSteps(const std::vector<float> xOffset, const std::vector<float> yOffset, RobotSide startLeg){

    ihmc_msgs::FootstepDataListRosMessage list;
    list.default_transfer_duration = transfer_time_;
    list.default_swing_duration = swing_time_;
    list.execution_mode = execution_mode_;
    list.unique_id = RobotWalker::id;

    if (xOffset.size() != yOffset.size()){
        ROS_ERROR("X Offset and Y Offset have different size");
        return false;
    }


    size_t numberOfSteps = xOffset.size();

    for (int m =1; m <= numberOfSteps ; m++) {
        if(m%2 == 1)
            list.footstep_data_list.push_back(*getOffsetStep(startLeg , xOffset[m-1], yOffset[m-1]));
        else
            list.footstep_data_list.push_back(*getOffsetStep((startLeg+1)%2 , xOffset[m-1], yOffset[m-1]));
    }

    this->walkGivenSteps(list);
    return true;
}

// walks predefined steps which could have varying step length and step widths. This is defined wrt Pelvis frame.
bool RobotWalker::walkLocalPreComputedSteps(const std::vector<float> xOffset, const std::vector<float> yOffset, RobotSide startLeg){

    ihmc_msgs::FootstepDataListRosMessage list;
    list.default_transfer_duration = transfer_time_;
    list.default_swing_duration = swing_time_;
    list.execution_mode = execution_mode_;
    list.unique_id = RobotWalker::id;

    if (xOffset.size() != yOffset.size())
        ROS_ERROR("X Offset and Y Offset have different size");

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    ihmc_msgs::FootstepDataRosMessage::Ptr newFootStep(new ihmc_msgs::FootstepDataRosMessage());

    geometry_msgs::Point currentWorldLocation,currentPelvisLocation;

    size_t numberOfSteps = xOffset.size();

    for (int m = 1; m <= numberOfSteps; ++m) {
        if(m%2 == 1)
        {
            getCurrentStep(startLeg, *current);
        }
        else
        {
            getCurrentStep((startLeg+1)%2, *current);
        }

        currentWorldLocation.x=current->location.x;
        currentWorldLocation.y=current->location.y;
        currentWorldLocation.z=current->location.z;
        current_state_->transformPoint(currentWorldLocation,currentPelvisLocation,TOUGH_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());

        currentPelvisLocation.x+=xOffset[m-1];
        currentPelvisLocation.y+=yOffset[m-1];
        current_state_->transformPoint(currentPelvisLocation,currentWorldLocation,rd_->getPelvisFrame(),TOUGH_COMMON_NAMES::WORLD_TF);
        newFootStep->location.x=currentWorldLocation.x;
        newFootStep->location.y=currentWorldLocation.y;
        newFootStep->location.z=current->location.z;
        newFootStep->orientation=current->orientation;
        newFootStep->robot_side=current->robot_side;
        newFootStep->trajectory_type=current->trajectory_type;
        list.footstep_data_list.push_back(*newFootStep);
    }

    this->walkGivenSteps(list);
    return true;
}

bool RobotWalker::walkLocalPreComputedSteps_waypoints(const std::vector<float> xOffset, const std::vector<float> yOffset, RobotSide startLeg){

    ihmc_msgs::FootstepDataListRosMessage list;
    list.default_transfer_duration= transfer_time_;
    list.default_swing_duration = swing_time_;
    list.execution_mode = execution_mode_;
    list.unique_id = RobotWalker::id;

    if (xOffset.size() != yOffset.size())
        ROS_ERROR("X Offset and Y Offset have different size");

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    ihmc_msgs::FootstepDataRosMessage::Ptr newFootStep(new ihmc_msgs::FootstepDataRosMessage());
    ihmc_msgs::FootstepDataRosMessage newFootStep1;

    geometry_msgs::Point currentWorldLocation,currentPelvisLocation;

    size_t numberOfSteps = xOffset.size();
    std::cout<<numberOfSteps<<std::endl;

    newFootStep->swing_trajectory.clear();
    newFootStep->unique_id = 678;

    newFootStep1.swing_trajectory.clear();
    newFootStep1.unique_id=10;

    ihmc_msgs::SE3TrajectoryPointRosMessage start, end; //,foot_trajectory2,foot_trajectory3;
    ihmc_msgs::SE3TrajectoryPointRosMessage::Ptr foot_trajectory1(new ihmc_msgs::SE3TrajectoryPointRosMessage());

    for (int m = 1; m <= numberOfSteps; ++m) {
        if(m%2 == 1)
        {
            getCurrentStep_waypoints(startLeg, *current);
        }
        else
        {
            getCurrentStep_waypoints((startLeg+1)%2, *current);
        }

        currentWorldLocation.x=current->location.x;
        currentWorldLocation.y=current->location.y;
        currentWorldLocation.z=current->location.z;

//        std::vector<ihmc_msgs::SE3TrajectoryPointRosMessage> foot_trajectories;

        start.position.x=currentWorldLocation.x;
        start.position.y=currentWorldLocation.y;
        start.position.z=currentWorldLocation.z;

        current_state_->transformPoint(currentWorldLocation,currentPelvisLocation,TOUGH_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());

        currentPelvisLocation.x+=xOffset[m-1];
        currentPelvisLocation.y+=yOffset[m-1];

        current_state_->transformPoint(currentPelvisLocation,currentWorldLocation,rd_->getPelvisFrame(),TOUGH_COMMON_NAMES::WORLD_TF);

        end.position.x=currentWorldLocation.x;
        end.position.y=currentWorldLocation.y;

        foot_trajectory1->position.x=0.25*(end.position.x-start.position.x)+start.position.x;
        foot_trajectory1->position.y=0.25*(end.position.y-start.position.y)+start.position.y;
        foot_trajectory1->position.z=start.position.z+0.10f;
        foot_trajectory1->orientation.x=0.0f;
        foot_trajectory1->orientation.y=-0.383f;
        foot_trajectory1->orientation.z=0.0f;
        foot_trajectory1->orientation.w=0.924f;
        //foot_trajectory1->time=0.25f;
        foot_trajectory1->linear_velocity.x=0.1f;
        foot_trajectory1->linear_velocity.y=0.001f;
        foot_trajectory1->linear_velocity.z=0.1f;
        foot_trajectory1->angular_velocity.x=0.001f;
        foot_trajectory1->angular_velocity.y=0.1f;
        foot_trajectory1->angular_velocity.z=0.0f;
        foot_trajectory1->unique_id=1653;

//        foot_trajectory2.position.x=0.50*(end.position.x-start.position.x)+start.position.x;
//        foot_trajectory2.position.y=0.50*(end.position.y-start.position.y)+start.position.y;
//        foot_trajectory2.position.z=start.position.z+0.15;
//        foot_trajectory2.orientation.x=0.0;
//        foot_trajectory2.orientation.y=-0.383;
//        foot_trajectory2.orientation.z=0.0;
//        foot_trajectory2.orientation.w=0.924;
//        foot_trajectory2.time=0.50f;
//        foot_trajectory2.linear_velocity.x=0.1;
//        foot_trajectory2.linear_velocity.y=0.0;
//        foot_trajectory2.linear_velocity.z=0.1;
//        foot_trajectory2.angular_velocity.x=0.0;
//        foot_trajectory2.angular_velocity.y=0.1;
//        foot_trajectory2.angular_velocity.z=0;
//        foot_trajectory2.unique_id=16523;

//        foot_trajectory3.position.x=0.75*(end.position.x-start.position.x)+start.position.x;
//        foot_trajectory3.position.y=0.75*(end.position.y-start.position.y)+start.position.y;
//        foot_trajectory3.position.z=start.position.z+0.10;
//        foot_trajectory3.orientation.x=0.0;
//        foot_trajectory3.orientation.y=-0.383;
//        foot_trajectory3.orientation.z=0.0;
//        foot_trajectory3.orientation.w=0.924;
//        foot_trajectory3.time=0.75f;
//        foot_trajectory3.linear_velocity.x=0.1;
//        foot_trajectory3.linear_velocity.y=0.0;
//        foot_trajectory3.linear_velocity.z=0.1;
//        foot_trajectory3.angular_velocity.x=0.0;
//        foot_trajectory3.angular_velocity.y=0.1;
//        foot_trajectory3.angular_velocity.z=0;
//        foot_trajectory3.unique_id=1523;

        newFootStep1.location.x=currentWorldLocation.x;
        newFootStep1.location.y=currentWorldLocation.y;
        newFootStep1.location.z=current->location.z;
        newFootStep1.orientation=current->orientation;
        newFootStep1.robot_side=current->robot_side;
        newFootStep1.trajectory_type=current->trajectory_type;

        newFootStep1.swing_trajectory.push_back(*foot_trajectory1);
//        newFootStep->swing_trajectory.push_back(foot_trajectory2);
//        newFootStep->swing_trajectory.push_back(foot_trajectory3);


        std::cout<<newFootStep1.swing_trajectory.size()<<std::endl;
        //newFootStep->swing_trajectory_blend_duration=0.1;
        newFootStep1.unique_id++;
        list.footstep_data_list.push_back(newFootStep1);
        std::cout<<list.footstep_data_list.size()<<std::endl;
    }

    this->walkGivenSteps(list);
    return true;
}

bool RobotWalker::walkGivenSteps(ihmc_msgs::FootstepDataListRosMessage& list , bool waitForSteps)
{
    this->footsteps_pub_.publish(list);
    RobotWalker::id--;
    if (waitForSteps){
        cbTime_=ros::Time::now();
        this->waitForSteps(list.footstep_data_list.size());
    }
    return true;
}
bool RobotWalker::raiseLeg(RobotSide side, float height,float stepLength)
{
    ihmc_msgs::FootTrajectoryRosMessage foot;
    ihmc_msgs::SE3TrajectoryPointRosMessage data;

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    getCurrentStep(side, *current);

    float stepFactor =1.0;
    // get current position
    data.position.x = current->location.x;
    data.position.y = current->location.y;
    data.position.z = current->location.z+(stepFactor*height);
    data.orientation=current->orientation;
    data.unique_id=100;
    data.time=2.0;
    foot.robot_side = side;
    foot.execution_mode=0; //OVERRIDE
    foot.unique_id=101;
    foot.taskspace_trajectory_points.push_back(data);



    //  take step forward
    // convert point to pelvis frame

    geometry_msgs::PointStamped pt_in,pt_out;
    pt_in.point.x=current->location.x;
    pt_in.point.y=current->location.y;
    pt_in.point.z=current->location.z;
    pt_in.header.frame_id=TOUGH_COMMON_NAMES::WORLD_TF;
    current_state_->transformPoint(pt_in,pt_out, rd_->getPelvisFrame());
//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::PELVIS_TF,TOUGH_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::PELVIS_TF, pt_in, pt_out);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }

    // add value of step length to x axis
    pt_out.point.x +=stepLength;

    // convert back to world frame
    current_state_->transformPoint(pt_out, pt_out);

//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::WORLD_TF,TOUGH_COMMON_NAMES::PELVIS_TF,ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::WORLD_TF, pt_out, pt_out);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }


    // add to data
    data.position.x = pt_out.point.x;
    data.position.z = current->location.z+(stepFactor*height); // tolerance
    data.unique_id=200;
    data.time=4.0;
    //      foot.taskspace_trajectory_points.push_back(data);

    //  take step down

    data.position.x = pt_out.point.x;
    data.position.z = current->location.z+height;
    data.unique_id=300;
    data.time=6.0;
    //    foot.taskspace_trajectory_points.push_back(data);

    nudgestep_pub_.publish(foot);

    return true;

}

bool RobotWalker::nudgeFoot(RobotSide side, float distance)
{
    ihmc_msgs::FootTrajectoryRosMessage foot;
    ihmc_msgs::SE3TrajectoryPointRosMessage data;

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    getCurrentStep(side, *current);

    geometry_msgs::PointStamped pt_in,pt_out;
    pt_in.point.x=current->location.x;
    pt_in.point.y=current->location.y;
    pt_in.point.z=current->location.z;
    pt_in.header.frame_id=TOUGH_COMMON_NAMES::WORLD_TF;
    current_state_->transformPoint(pt_in,pt_out, rd_->getPelvisFrame());
//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::PELVIS_TF,TOUGH_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::PELVIS_TF, pt_in, pt_out);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }

    // convert back to world frame
    pt_out.point.x+=distance;
    current_state_->transformPoint(pt_out, pt_out);
//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::WORLD_TF,TOUGH_COMMON_NAMES::PELVIS_TF,ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::WORLD_TF, pt_out, pt_out);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }


    // add to data

    data.position.x = pt_out.point.x;
    data.position.y = pt_out.point.y;
    data.position.z = pt_out.point.z;
    data.orientation=current->orientation;

    std::cout<<"point x"<<data.position.x<<"\n";
    std::cout<<"point y"<<data.position.y<<"\n";
    std::cout<<"point z"<<data.position.z<<"\n";

    data.unique_id=700;
    data.time=2.0;

    foot.robot_side = side;
    foot.execution_mode=0; //OVERRIDE
    foot.unique_id=321;
    foot.taskspace_trajectory_points.push_back(data);

    nudgestep_pub_.publish(foot);

    return true;


}

bool RobotWalker::curlLeg(RobotSide side, float radius)
{
    ihmc_msgs::FootTrajectoryRosMessage foot;
    ihmc_msgs::SE3TrajectoryPointRosMessage data;

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    getCurrentStep(side, *current);

    // converting point in pelvis frame
    geometry_msgs::PointStamped pt_in,pt_out;
    geometry_msgs::Pose final;
    pt_in.point.x=current->location.x;
    pt_in.point.y=current->location.y;
    pt_in.point.z=current->location.z;
    pt_in.header.frame_id=TOUGH_COMMON_NAMES::WORLD_TF;
    current_state_->transformPoint(pt_in,pt_out, rd_->getPelvisFrame());

//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::PELVIS_TF,TOUGH_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::PELVIS_TF, pt_in, pt_out);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }

    // converting back to world frame
    final.position.x=pt_out.point.x-radius;
    final.position.y=pt_out.point.y;
    final.position.z=pt_out.point.z+radius;
    final.orientation.x=0;
    final.orientation.y=0.5;
    final.orientation.z=0;
    final.orientation.w=0.866;
//    final.header.frame_id =rd_->getPelvisFrame();
//    final.header.stamp =ros::Time(0);

    current_state_->transformPose(final,final,rd_->getPelvisFrame());
//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::PELVIS_TF,TOUGH_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPose(TOUGH_COMMON_NAMES::WORLD_TF, final, final);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }

    // get current position
    data.position.x = final.position.x;
    data.position.y = final.position.y;
    data.position.z = final.position.z;
    data.orientation=final.orientation;
    data.unique_id=100;
    data.time=3.0;
    foot.robot_side = side;
    foot.execution_mode=0; //OVERRIDE
    foot.unique_id=101;
    foot.taskspace_trajectory_points.push_back(data);
    nudgestep_pub_.publish(foot);
    return true;

}

bool RobotWalker::placeLeg(RobotSide side, float offset)
{

    ihmc_msgs::FootTrajectoryRosMessage foot;
    ihmc_msgs::SE3TrajectoryPointRosMessage data;

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    getCurrentStep((side+1)%2, *current);

    geometry_msgs::PointStamped pt_in,pt_out;
    pt_in.point.x=current->location.x;
    pt_in.point.y=current->location.y;
    pt_in.point.z=current->location.z;
    pt_in.header.frame_id=TOUGH_COMMON_NAMES::WORLD_TF;
    current_state_->transformPoint(pt_in,pt_out, rd_->getPelvisFrame());

//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::PELVIS_TF,TOUGH_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::PELVIS_TF, pt_in, pt_out);

//    }
//   //void RobotWalker::getCurrentStep_waypoints(int side , ihmc_msgs::FootstepDataRosMessage & foot)
//{

//    std_msgs::String foot_frame =  side == LEFT ? left_foot_frame_ : right_foot_frame_;

//    tf::StampedTransform transformStamped;

//    /// \todo Use a try catch block here. It needs modification of function
//    /// signature to return bool and all functions in the heirarchy would be changed accordingly.
//    //    tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, foot_frame,ros::Time(0), ros::Duration(2.0));
//    tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF,foot_frame.data,ros::Time(0),transformStamped);

//    tf::quaternionTFToMsg(transformStamped.getRotation(),foot.orientation);
//    foot.location.x = transformStamped.getOrigin().getX();
//    foot.location.y = transformStamped.getOrigin().getY();
//    foot.location.z = transformStamped.getOrigin().getZ();
//    foot.robot_side = side;
//    foot.trajectory_type = ihmc_msgs::FootstepDataRosMessage::WAYPOINTS;
//    return;
//} catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }

    // convert back to world frame
    if(side ==LEFT)
    {
        pt_out.point.y+=0.20;
    }
    else pt_out.point.y-=0.20;

    pt_out.point.z+=offset;
    current_state_->transformPoint(pt_out, pt_out);

//    try{

//        tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::WORLD_TF,TOUGH_COMMON_NAMES::PELVIS_TF,ros::Time(0),ros::Duration(2));
//        tf_listener_.transformPoint(TOUGH_COMMON_NAMES::WORLD_TF, pt_out, pt_out);

//    }
//    catch (tf::TransformException ex){
//        ROS_WARN("%s",ex.what());
//        ros::spinOnce();
//        return false;
//    }


    // add to data

    data.position.x = pt_out.point.x;
    data.position.y = pt_out.point.y;
    data.position.z = pt_out.point.z;
    data.orientation=current->orientation;

    std::cout<<"point x "<<data.position.x<<"\n";
    std::cout<<"point y "<<data.position.y<<"\n";
    std::cout<<"point z "<<data.position.z<<"\n";

    data.unique_id=900;
    data.time=3.0;

    foot.robot_side = side;
    foot.execution_mode=0; //OVERRIDE
    foot.unique_id=391;
    foot.taskspace_trajectory_points.push_back(data);

    nudgestep_pub_.publish(foot);

    return true;

}

//Calls the footstep planner service to get footsteps to reach goal
bool RobotWalker::getFootstep(geometry_msgs::Pose2D &goal,ihmc_msgs::FootstepDataListRosMessage &list)
{
    /// \todo fix the robot pose, if the legs are not together before walking.

    geometry_msgs::Pose2D start;
    humanoid_nav_msgs::PlanFootsteps srv;
    footstep_client_ = nh_.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("/plan_footsteps");
    // get start from robot position

    //    ihmc_msgs::FootstepDataRosMessage::Ptr startstep(new ihmc_msgs::FootstepDataRosMessage());
    //this->getCurrentStep(0,*startstep);
    geometry_msgs::Pose pelvisPose, leftFootPose, rightFootPose;
    current_state_->getCurrentPose(rd_->getLeftFootFrameName(),leftFootPose);
    current_state_->getCurrentPose(rd_->getRightFootFrameName(), rightFootPose);

    start.x = (leftFootPose.position.x + rightFootPose.position.x)/2.0f;
    start.y = (leftFootPose.position.y + rightFootPose.position.y)/2.0f; // This is required to offset the left foot to get senter of the
    //    std::cout<< "Start Position  x = " << start.x << "  y = " << start.y<<std::endl;

    start.theta = tf::getYaw(rightFootPose.orientation);

    srv.request.start = start;
    srv.request.goal = goal;
    // The service calls succeeds everytime. result variable stores the actual result of planning
    if(footstep_client_.call(srv) && srv.response.result)
    {

        for(int i=0; i <srv.response.footsteps.size();i++)
        {

            ihmc_msgs::FootstepDataRosMessage::Ptr step(new ihmc_msgs::FootstepDataRosMessage());
            bool side = bool(srv.response.footsteps.at(i).leg);

            side = !side;

            this->getCurrentStep(int(side),*step);

            step->location.x = srv.response.footsteps.at(i).pose.x;
            step->location.y = srv.response.footsteps.at(i).pose.y;
            step->location.z = step->location.z;

            tf::Quaternion t = tf::createQuaternionFromYaw(srv.response.footsteps.at(i).pose.theta);
            ROS_DEBUG("Step x  %d %.2f", i, srv.response.footsteps.at(i).pose.x);
            ROS_DEBUG("Step y  %d %.2f", i, srv.response.footsteps.at(i).pose.y);
            ROS_DEBUG("Side  %d %d",i, int(side));

            step->orientation.w = t.w();
            step->orientation.x = t.x();
            step->orientation.y = t.y();
            step->orientation.z = t.z();
            
            list.footstep_data_list.push_back(*step);
        }
        return true;
    }
    return false;
}

double RobotWalker::getSwingHeight() const
{
    return swing_height_;
}


// Get starting location of the foot

void RobotWalker::getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage & foot)
{

    std_msgs::String foot_frame =  side == LEFT ? left_foot_frame_ : right_foot_frame_;

    tf::StampedTransform transformStamped;

    /// \todo Use a try catch block here. It needs modification of function
    /// signature to return bool and all functions in the heirarchy would be changed accordingly.
    //    tf_listener_.waitForTransform(TOUGH_COMMON_NAMES::WORLD_TF, foot_frame,ros::Time(0), ros::Duration(2.0));
    tf_listener_.lookupTransform( TOUGH_COMMON_NAMES::WORLD_TF,foot_frame.data,ros::Time(0),transformStamped);

    tf::quaternionTFToMsg(transformStamped.getRotation(),foot.orientation);
    foot.location.x = transformStamped.getOrigin().getX();
    foot.location.y = transformStamped.getOrigin().getY();
    foot.location.z = transformStamped.getOrigin().getZ();
    foot.robot_side = side;
    foot.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
    return;
}

void RobotWalker::getCurrentStep_waypoints(int side , ihmc_msgs::FootstepDataRosMessage & foot)
{

    std_msgs::String foot_frame =  side == LEFT ? left_foot_frame_ : right_foot_frame_;

    tf::StampedTransform transformStamped;

    /// \todo Use a try catch block here. It needs modification of function
    /// signature to return bool and all functions in the heirarchy would be changed accordingly.
    //    tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, foot_frame,ros::Time(0), ros::Duration(2.0));
    tf_listener_.lookupTransform( TOUGH_COMMON_NAMES::WORLD_TF,foot_frame.data,ros::Time(0),transformStamped);

    tf::quaternionTFToMsg(transformStamped.getRotation(),foot.orientation);
    foot.location.x = transformStamped.getOrigin().getX();
    foot.location.y = transformStamped.getOrigin().getY();
    foot.location.z = transformStamped.getOrigin().getZ();
    foot.robot_side = side;
    foot.trajectory_type = ihmc_msgs::FootstepDataRosMessage::WAYPOINTS;
    return;
}

// gives footstep which are offset from current step (only for straight line)

ihmc_msgs::FootstepDataRosMessage::Ptr RobotWalker::getOffsetStep(int side , float x, float y)
{

    ihmc_msgs::FootstepDataRosMessage::Ptr next(new ihmc_msgs::FootstepDataRosMessage());

    this->getCurrentStep(side, *next);
    next->location.x+=x;
    next->location.y+=y;
    next->swing_height = swing_height_;
    return next;

}

ihmc_msgs::FootstepDataRosMessage::Ptr RobotWalker::getOffsetStepWRTPelvis(int side , float x, float y)
{

    ihmc_msgs::FootstepDataRosMessage::Ptr next(new ihmc_msgs::FootstepDataRosMessage());
    geometry_msgs::Point currentWorldLocation,currentPelvisLocation;

    // get the current step
    getCurrentStep(side, *next);
    currentWorldLocation.x=next->location.x;
    currentWorldLocation.y=next->location.y;
    currentWorldLocation.z=next->location.z;

    // transform the step to pelvis
    current_state_->transformPoint(currentWorldLocation,currentPelvisLocation,TOUGH_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
    // add the offsets wrt to pelvis
    currentPelvisLocation.x+=x;
    currentPelvisLocation.y+=y;
    // tranform back the point to plevis
    current_state_->transformPoint(currentPelvisLocation,currentWorldLocation,rd_->getPelvisFrame(),TOUGH_COMMON_NAMES::WORLD_TF);

    // update the new location
    next->location.x=currentWorldLocation.x;
    next->location.y=currentWorldLocation.y;
    next->location.z=currentWorldLocation.z;
    next->swing_height = swing_height_;

    // return it
    return next;
}

bool RobotWalker::turn(RobotSide side)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_duration = transfer_time_;
    list.default_swing_duration    = swing_time_;
    list.execution_mode        = execution_mode_;
    list.unique_id             = RobotWalker::id;

    ihmc_msgs::FootstepDataRosMessage step;

    geometry_msgs::PointStamped world_values;
    geometry_msgs::QuaternionStamped world_quat;

    std::vector<geometry_msgs::PointStamped>    pelvis_valuesL;
    std::vector<geometry_msgs::QuaternionStamped> pelvis_quatL;
    std::vector<geometry_msgs::PointStamped>    pelvis_valuesR;
    std::vector<geometry_msgs::QuaternionStamped> pelvis_quatR;

    // resizing vector based on number of footsteps
    pelvis_valuesL.resize(5);
    pelvis_valuesR.resize(5);
    pelvis_quatL.resize(5);
    pelvis_quatR.resize(5);

    // Left
    pelvis_valuesL[0].point.x= 0.135900;
    pelvis_valuesL[0].point.y= -0.154233;
    pelvis_valuesL[0].point.z= 0.086523;
    pelvis_quatL[0].quaternion.w= 0.980231;
    pelvis_quatL[0].quaternion.x= 0.000016;
    pelvis_quatL[0].quaternion.y= 0.000160;
    pelvis_quatL[0].quaternion.z= 0.197856;

    pelvis_valuesL[1].point.x= -0.055072;
    pelvis_valuesL[1].point.y= 0.034846;
    pelvis_valuesL[1].point.z= 0.086575;
    pelvis_quatL[1].quaternion.w= 0.941065;
    pelvis_quatL[1].quaternion.x= 0.000039;
    pelvis_quatL[1].quaternion.y= 0.000155;
    pelvis_quatL[1].quaternion.z= 0.338227;

    pelvis_valuesL[2].point.x= 0.105901;
    pelvis_valuesL[2].point.y= -0.154402;
    pelvis_valuesL[2].point.z= 0.086533;
    pelvis_quatL[2].quaternion.w= 0.902780;
    pelvis_quatL[2].quaternion.x= 0.000055;
    pelvis_quatL[2].quaternion.y= 0.000151;
    pelvis_quatL[2].quaternion.z= 0.430103;

    pelvis_valuesL[3].point.x= -0.084986;
    pelvis_valuesL[3].point.y= 0.004761;
    pelvis_valuesL[3].point.z= 0.086586;
    pelvis_quatL[3].quaternion.w= 0.706102;
    pelvis_quatL[3].quaternion.x= 0.000101;
    pelvis_quatL[3].quaternion.y= 0.000123;
    pelvis_quatL[3].quaternion.z= 0.708111;

    pelvis_valuesL[4].point.x= 0.090027;
    pelvis_valuesL[4].point.y= 0.000439;
    pelvis_valuesL[4].point.z= 0.086533;
    pelvis_quatL[4].quaternion.w= 0.705392;
    pelvis_quatL[4].quaternion.x= 0.000102;
    pelvis_quatL[4].quaternion.y= 0.000124;
    pelvis_quatL[4].quaternion.z= 0.708818;

    // Right

    pelvis_valuesR[0].point.x= -0.064089;
    pelvis_valuesR[0].point.y= -0.145396;
    pelvis_valuesR[0].point.z= 0.078471;
    pelvis_quatR[0].quaternion.w= 0.981378;
    pelvis_quatR[0].quaternion.x= -0.000023;
    pelvis_quatR[0].quaternion.y= 0.000136;
    pelvis_quatR[0].quaternion.z= -0.192087;

    pelvis_valuesR[1].point.x= 0.034601;
    pelvis_valuesR[1].point.y= 0.085171;
    pelvis_valuesR[1].point.z= 0.078451;
    pelvis_quatR[1].quaternion.w= 0.924822;
    pelvis_quatR[1].quaternion.x= -0.000049;
    pelvis_quatR[1].quaternion.y= 0.000129;
    pelvis_quatR[1].quaternion.z= -0.380401;

    pelvis_valuesR[2].point.x= -0.144394;
    pelvis_valuesR[2].point.y= -0.095886;
    pelvis_valuesR[2].point.z= 0.078494;
    pelvis_quatR[2].quaternion.w= 0.833166;
    pelvis_quatR[2].quaternion.x= -0.000074;
    pelvis_quatR[2].quaternion.y= 0.000117;
    pelvis_quatR[2].quaternion.z= -0.553023;

    pelvis_valuesR[3].point.x= 0.084996;
    pelvis_valuesR[3].point.y= 0.005419;
    pelvis_valuesR[3].point.z= 0.078437;
    pelvis_quatR[3].quaternion.w= 0.708850;
    pelvis_quatR[3].quaternion.x= -0.000094;
    pelvis_quatR[3].quaternion.y= 0.000100;
    pelvis_quatR[3].quaternion.z= -0.705359;

    pelvis_valuesR[4].point.x= -0.089976;
    pelvis_valuesR[4].point.y= -0.000623;
    pelvis_valuesR[4].point.z= 0.078479;
    pelvis_quatR[4].quaternion.w= 0.709548;
    pelvis_quatR[4].quaternion.x= -0.000095;
    pelvis_quatR[4].quaternion.y= 0.000100;
    pelvis_quatR[4].quaternion.z= -0.704657;


    for (int i = 0; i < pelvis_valuesL.size(); ++i) {
        pelvis_valuesL[i].header.frame_id=rd_->getPelvisFrame();
        pelvis_quatL[i].header.frame_id=rd_->getPelvisFrame();
    }

    for (int i = 0; i < pelvis_valuesR.size(); ++i) {
        pelvis_valuesR[i].header.frame_id=rd_->getPelvisFrame();
        pelvis_quatR[i].header.frame_id=rd_->getPelvisFrame();
    }


    if(side==LEFT)
    {
        for (int i = 0; i < pelvis_valuesL.size(); ++i) {

            current_state_->transformPoint(pelvis_valuesL[i], world_values);
            current_state_->transformQuaternion(pelvis_quatL[i], world_quat);

            step.robot_side=i%2 ? LEFT : RIGHT;
            step.location=world_values.point;
            step.orientation=world_quat.quaternion;

            list.footstep_data_list.push_back(step);

        }
    }
    else
    {
        for (int i = 0; i < pelvis_valuesR.size(); ++i) {

            current_state_->transformPoint(pelvis_valuesR[i], world_values);
            current_state_->transformQuaternion(pelvis_quatR[i], world_quat);

            step.robot_side=i%2 ? LEFT : RIGHT;
            step.location=world_values.point;
            step.orientation=world_quat.quaternion;

            list.footstep_data_list.push_back(step);

        }
    }

    // publish footsteps

    this->footsteps_pub_.publish(list);
    RobotWalker::id--;
    this->waitForSteps(list.footstep_data_list.size());

}

void RobotWalker::loadEEF(RobotSide side, EE_LOADING load)
{
    ihmc_msgs::EndEffectorLoadBearingRosMessage msg;
    msg.unique_id=1;
    msg.robot_side=side;
    msg.end_effector=0;  // 0- foot 1 -hand
    msg.request=(int)load;  // 0 -load 1 -unload
    loadeff_pub.publish(msg);

}

bool RobotWalker::walkRotate(float angle)
{

    // get current position
    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    getCurrentStep(RIGHT, *current);

    geometry_msgs::Pose2D goal;
    goal.x = current->location.x;
    goal.y = current->location.y;
    goal.theta = tf::getYaw(current->orientation)+angle;
    walkToGoal(goal);
    return true;

}

bool RobotWalker::climbStair(const std::vector<float> xOffset, const std::vector<float> zOffset, RobotSide startLeg)
{
    ihmc_msgs::FootstepDataListRosMessage list;
    list.default_transfer_duration= transfer_time_;
    list.default_swing_duration = swing_time_;
    list.execution_mode = execution_mode_;
    list.unique_id = RobotWalker::id;

    geometry_msgs::Point waypoint;
    float offset =0.1;

    if (xOffset.size() != zOffset.size())
        ROS_ERROR("X Offset and Z Offset have different size");

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    ihmc_msgs::FootstepDataRosMessage::Ptr newFootStep(new ihmc_msgs::FootstepDataRosMessage());

    geometry_msgs::Point currentWorldLocation,currentPelvisLocation;

    size_t numberOfSteps = xOffset.size();

    for (int m = 1; m <= numberOfSteps; ++m) {
        if(m%2 == 1)
        {
            getCurrentStep(startLeg, *current);
        }
        else
        {
            getCurrentStep((startLeg+1)%2, *current);
        }

        currentWorldLocation =current->location;

        current_state_->transformPoint(currentWorldLocation,currentPelvisLocation,TOUGH_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
        currentPelvisLocation.x+=xOffset[m-1];
        currentPelvisLocation.z+=zOffset[m-1];
        current_state_->transformPoint(currentPelvisLocation,currentWorldLocation,rd_->getPelvisFrame(),TOUGH_COMMON_NAMES::WORLD_TF);
        newFootStep->location.x=currentWorldLocation.x;
        newFootStep->location.y=currentWorldLocation.y;
        newFootStep->location.z=currentWorldLocation.z;
        newFootStep->orientation=current->orientation;
        newFootStep->robot_side=current->robot_side;
        newFootStep->trajectory_type=1; // 0 - DEFAULT, 1 - OBSTACLE CLEARANCE, 2- CUSTOM

        currentWorldLocation.x=current->location.x;
        currentWorldLocation.y=current->location.y;
        currentWorldLocation.z=current->location.z;

        if(m>2)
        {
            current_state_->transformPoint(currentWorldLocation,currentPelvisLocation,TOUGH_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
            currentPelvisLocation.x+=(xOffset[m-2]-0.1);
            currentPelvisLocation.z+=zOffset[m-1]+offset;
            current_state_->transformPoint(currentPelvisLocation,currentWorldLocation,rd_->getPelvisFrame(),TOUGH_COMMON_NAMES::WORLD_TF);

            newFootStep->position_waypoints.push_back(currentWorldLocation);
            newFootStep->trajectory_type=2; // 0 - DEFAULT, 1 - OBSTACLE CLEARANCE, 2- CUSTOM
        }


        newFootStep->position_waypoints.push_back(waypoint);
        list.footstep_data_list.push_back(*newFootStep);
    }

    this->walkGivenSteps(list);
    return true;
}

// wait till all the steps are taken
void RobotWalker::waitForSteps(int numSteps)
{
    while (step_counter_ < numSteps && ros::ok())
    {
        ros::spinOnce();

        // hack to detect if robot has fallen and to exit this block
        if ((ros::Time::now() - cbTime_) > ros::Duration(5))
        {
//            ROS_INFO("robot fallen, exiting");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    // reset back the counter
    step_counter_ = 0;
    return;
}
