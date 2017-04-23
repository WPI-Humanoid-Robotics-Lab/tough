/*
 *
 *
 *
 * */

#include "val_footstep/ValkyrieWalker.h"
#include "val_common/val_common_names.h"
#include <iostream>
#include <ros/ros.h>

int ValkyrieWalker::id = -1;

// CallBack function for walking status
void ValkyrieWalker::footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage & msg)
{
    if(msg.status == 1)
    {
        step_counter++;
        //ROS_INFO("step counter : %d",step_counter);
    }

    // reset the timer
    cbTime_=ros::Time::now();

    return;
}

// creates and send footsteps to  val to reach goal position
bool ValkyrieWalker::walkToGoal( geometry_msgs::Pose2D &goal, bool waitForSteps)
{    
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_time = transfer_time;
    list.default_swing_time= swing_time;
    list.execution_mode = exe_mode;
    list.unique_id = ValkyrieWalker::id;

    if(this->getFootstep(goal,list))
    {
        this->footsteps_pub_.publish(list);
        ValkyrieWalker::id--;

        if (waitForSteps)
        {
            this->waitForSteps(list.footstep_data_list.size());
        }
        return true;
    }
    return false;
}

// creates and n footsteps of width step_size
bool ValkyrieWalker::walkNSteps(int n, float x_offset, float y_offset, bool continous, armSide startLeg, bool waitForSteps)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_time = transfer_time;
    list.default_swing_time = swing_time;
    list.execution_mode = exe_mode;

    list.unique_id = ValkyrieWalker::id ;

    for (int m =1; m <= n ; m++) {
        if(m%2 == 1) {
            list.footstep_data_list.push_back(*getOffsetStep(startLeg , m*x_offset, m*y_offset));
        }
        else {
            list.footstep_data_list.push_back(*getOffsetStep((startLeg+1)%2 , m*x_offset, m*y_offset));
        }

    }
    if(!continous){
        if (n%2 ==1) {
            list.footstep_data_list.push_back(*getOffsetStep((startLeg+1)%2  , n*x_offset, n*y_offset));
        }
        if (n%2 ==0) {
            list.footstep_data_list.push_back(*getOffsetStep(startLeg , n*x_offset, n*y_offset));
        }
    }

    this->walkGivenSteps(list, waitForSteps);
    return true;
}

bool ValkyrieWalker::walkPreComputedSteps(const std::vector<float> x_offset, const std::vector<float> y_offset, armSide startLeg){

    ihmc_msgs::FootstepDataListRosMessage list;
    list.default_transfer_time= transfer_time;
    list.default_swing_time = swing_time;
    list.execution_mode = exe_mode;
    list.unique_id = ValkyrieWalker::id;

    if (x_offset.size() != y_offset.size())
        ROS_ERROR("X Offset and Y Offset have different size");


    size_t numberOfStpes = x_offset.size();

    for (int m =1; m <= numberOfStpes ; m++) {
        if(m%2 == 1)
            list.footstep_data_list.push_back(*getOffsetStep(startLeg , x_offset[m-1], y_offset[m-1]));
        else
            list.footstep_data_list.push_back(*getOffsetStep((startLeg+1)%2 , x_offset[m-1], y_offset[m-1]));
    }

    this->walkGivenSteps(list);
    return true;
}


bool ValkyrieWalker::walkGivenSteps(ihmc_msgs::FootstepDataListRosMessage& list , bool waitForSteps)
{
    this->footsteps_pub_.publish(list);
    ValkyrieWalker::id--;
    if (waitForSteps){
        this->waitForSteps(list.footstep_data_list.size());
    }
    return true;
}


//Calls the footstep planner service to get footsteps to reach goal
bool ValkyrieWalker::getFootstep(geometry_msgs::Pose2D &goal,ihmc_msgs::FootstepDataListRosMessage &list)
{
    /// \todo fix the robot pose, if the legs are not together before walking.

    geometry_msgs::Pose2D start;
    humanoid_nav_msgs::PlanFootsteps srv;

    // get start from robot position

    ihmc_msgs::FootstepDataRosMessage* startstep = new ihmc_msgs::FootstepDataRosMessage() ;
    this->getCurrentStep(0,*startstep);


    start.x = startstep->location.x ;
    start.y = startstep->location.y; // - 0.18; Why do we need to subtract this?
    //    std::cout<< "Start Position  x = " << start.x << "  y = " << start.y<<std::endl;

    start.theta = tf::getYaw(startstep->orientation);

    delete startstep;


    srv.request.start = start;
    srv.request.goal = goal;

    if(this->footstep_client_.call(srv))
    {

        for(int i=0; i <srv.response.footsteps.size();i++)
        {

            ///\ todo shared pointer implementation
            ihmc_msgs::FootstepDataRosMessage* step = new ihmc_msgs::FootstepDataRosMessage() ;
            bool side = bool(srv.response.footsteps.at(i).leg);

            side = !side;

            this->getCurrentStep(int(side),*step);

            step->location.x = srv.response.footsteps.at(i).pose.x;
            step->location.y = srv.response.footsteps.at(i).pose.y;


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

double ValkyrieWalker::getSwing_height() const
{
    return swing_height;
}




// constructor
ValkyrieWalker::ValkyrieWalker(ros::NodeHandle nh,double InTransferTime ,double InSwingTime, int InMode, double swingHeight):nh_(nh)
{
    this->footstep_client_ = nh_.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("/plan_footsteps");
    this->footsteps_pub_   = nh_.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
    this->footstep_status_ = nh_.subscribe("/ihmc_ros/valkyrie/output/footstep_status", 20,&ValkyrieWalker::footstepStatusCB, this);

    transfer_time = InTransferTime;
    swing_time = InSwingTime;
    exe_mode = InMode;
    swing_height = swingHeight;

    ros::Duration(0.5).sleep();
    step_counter = 0;
    ValkyrieWalker::id = -1;

    //start timer
    cbTime_=ros::Time::now();
    std::string robot_name;

    if (nh_.getParam("/ihmc_ros/robot_name",robot_name))
    {
        if(nh_.getParam("/ihmc_ros/valkyrie/right_foot_frame_name", right_foot_frame_.data) && nh_.getParam("/ihmc_ros/valkyrie/left_foot_frame_name", left_foot_frame_.data))
        {
            ROS_DEBUG("%s", right_foot_frame_.data.c_str());
            ROS_DEBUG("%s", left_foot_frame_.data.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to get param foot frames.");
    }
}
// Destructor
ValkyrieWalker::~ValkyrieWalker(){
}

// Get starting location of the foot

void ValkyrieWalker::getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage & foot)
{

    std_msgs::String foot_frame;
    if (side == LEFT)
    {
        foot_frame = this->left_foot_frame_;
    }
    else
    {
        foot_frame = this->right_foot_frame_;
    }

    tf::StampedTransform transformStamped;
    tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF,foot_frame.data,ros::Time(0),transformStamped);
    tf::quaternionTFToMsg(transformStamped.getRotation(),foot.orientation);
    foot.location.x = transformStamped.getOrigin().getX();
    foot.location.y = transformStamped.getOrigin().getY();
    foot.location.z = transformStamped.getOrigin().getZ();
    foot.robot_side = side;
    foot.trajectory_type = 0;
    return;
}

// gives footstep which are offset from current step (only for straight line)

ihmc_msgs::FootstepDataRosMessage* ValkyrieWalker::getOffsetStep(int side , float x, float y)
{
    ///\ todo shared pointer implementation
    ihmc_msgs::FootstepDataRosMessage *next = new ihmc_msgs::FootstepDataRosMessage();

    this->getCurrentStep(side, *next);
    next->location.x+=x;
    next->location.y+=y;
    next->swing_height = swing_height;
    return next;

}

bool ValkyrieWalker::turn(armSide side)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.default_transfer_time = transfer_time;
    list.default_swing_time= swing_time;
    list.execution_mode = exe_mode;
    list.unique_id = ValkyrieWalker::id;
    ihmc_msgs::FootstepDataRosMessage step;

    geometry_msgs::Vector3Stamped world_values;
    geometry_msgs::QuaternionStamped world_quat;

    std::vector<geometry_msgs::Vector3Stamped> pelvis_valuesL;
    std::vector<geometry_msgs::QuaternionStamped> pelvis_quatL;
    std::vector<geometry_msgs::Vector3Stamped> pelvis_valuesR;
    std::vector<geometry_msgs::QuaternionStamped> pelvis_quatR;

    // resizing vector based on number of footsteps
    pelvis_valuesL.resize(5);
    pelvis_valuesR.resize(5);
    pelvis_quatL.resize(5);
    pelvis_quatR.resize(5);

    // Left
    pelvis_valuesL[0].vector.x= 0.135900;
    pelvis_valuesL[0].vector.y= -0.154233;
    pelvis_valuesL[0].vector.z= 0.086523;
    pelvis_quatL[0].quaternion.w= 0.980231;
    pelvis_quatL[0].quaternion.x= 0.000016;
    pelvis_quatL[0].quaternion.y= 0.000160;
    pelvis_quatL[0].quaternion.z= 0.197856;

    pelvis_valuesL[1].vector.x= -0.055072;
    pelvis_valuesL[1].vector.y= 0.034846;
    pelvis_valuesL[1].vector.z= 0.086575;
    pelvis_quatL[1].quaternion.w= 0.941065;
    pelvis_quatL[1].quaternion.x= 0.000039;
    pelvis_quatL[1].quaternion.y= 0.000155;
    pelvis_quatL[1].quaternion.z= 0.338227;

    pelvis_valuesL[2].vector.x= 0.105901;
    pelvis_valuesL[2].vector.y= -0.154402;
    pelvis_valuesL[2].vector.z= 0.086533;
    pelvis_quatL[2].quaternion.w= 0.902780;
    pelvis_quatL[2].quaternion.x= 0.000055;
    pelvis_quatL[2].quaternion.y= 0.000151;
    pelvis_quatL[2].quaternion.z= 0.430103;

    pelvis_valuesL[3].vector.x= -0.084986;
    pelvis_valuesL[3].vector.y= 0.004761;
    pelvis_valuesL[3].vector.z= 0.086586;
    pelvis_quatL[3].quaternion.w= 0.706102;
    pelvis_quatL[3].quaternion.x= 0.000101;
    pelvis_quatL[3].quaternion.y= 0.000123;
    pelvis_quatL[3].quaternion.z= 0.708111;

    pelvis_valuesL[4].vector.x= 0.090027;
    pelvis_valuesL[4].vector.y= 0.000439;
    pelvis_valuesL[4].vector.z= 0.086533;
    pelvis_quatL[4].quaternion.w= 0.705392;
    pelvis_quatL[4].quaternion.x= 0.000102;
    pelvis_quatL[4].quaternion.y= 0.000124;
    pelvis_quatL[4].quaternion.z= 0.708818;

    // Right

    pelvis_valuesR[0].vector.x= -0.064089;
    pelvis_valuesR[0].vector.y= -0.145396;
    pelvis_valuesR[0].vector.z= 0.078471;
    pelvis_quatR[0].quaternion.w= 0.981378;
    pelvis_quatR[0].quaternion.x= -0.000023;
    pelvis_quatR[0].quaternion.y= 0.000136;
    pelvis_quatR[0].quaternion.z= -0.192087;

    pelvis_valuesR[1].vector.x= 0.034601;
    pelvis_valuesR[1].vector.y= 0.085171;
    pelvis_valuesR[1].vector.z= 0.078451;
    pelvis_quatR[1].quaternion.w= 0.924822;
    pelvis_quatR[1].quaternion.x= -0.000049;
    pelvis_quatR[1].quaternion.y= 0.000129;
    pelvis_quatR[1].quaternion.z= -0.380401;

    pelvis_valuesR[2].vector.x= -0.144394;
    pelvis_valuesR[2].vector.y= -0.095886;
    pelvis_valuesR[2].vector.z= 0.078494;
    pelvis_quatR[2].quaternion.w= 0.833166;
    pelvis_quatR[2].quaternion.x= -0.000074;
    pelvis_quatR[2].quaternion.y= 0.000117;
    pelvis_quatR[2].quaternion.z= -0.553023;

    pelvis_valuesR[3].vector.x= 0.084996;
    pelvis_valuesR[3].vector.y= 0.005419;
    pelvis_valuesR[3].vector.z= 0.078437;
    pelvis_quatR[3].quaternion.w= 0.708850;
    pelvis_quatR[3].quaternion.x= -0.000094;
    pelvis_quatR[3].quaternion.y= 0.000100;
    pelvis_quatR[3].quaternion.z= -0.705359;

    pelvis_valuesR[4].vector.x= -0.089976;
    pelvis_valuesR[4].vector.y= -0.000623;
    pelvis_valuesR[4].vector.z= 0.078479;
    pelvis_quatR[4].quaternion.w= 0.709548;
    pelvis_quatR[4].quaternion.x= -0.000095;
    pelvis_quatR[4].quaternion.y= 0.000100;
    pelvis_quatR[4].quaternion.z= -0.704657;


    for (int i = 0; i < pelvis_valuesL.size(); ++i) {
        pelvis_valuesL[i].header.frame_id=VAL_COMMON_NAMES::PELVIS_TF;
        pelvis_quatL[i].header.frame_id=VAL_COMMON_NAMES::PELVIS_TF;
    }

    for (int i = 0; i < pelvis_valuesR.size(); ++i) {
        pelvis_valuesR[i].header.frame_id=VAL_COMMON_NAMES::PELVIS_TF;
        pelvis_quatR[i].header.frame_id=VAL_COMMON_NAMES::PELVIS_TF;
    }


    if(side==LEFT)
    {
        for (int i = 0; i < pelvis_valuesL.size(); ++i) {

            try{
                tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF, ros::Time(0), ros::Duration(3.0));
                tf_listener_.transformVector(VAL_COMMON_NAMES::WORLD_TF,pelvis_valuesL[i],world_values);
                tf_listener_.transformQuaternion(VAL_COMMON_NAMES::WORLD_TF,pelvis_quatL[i],world_quat);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                return false;
            }

            step.robot_side=i%2 ? LEFT : RIGHT;
            step.location=world_values.vector;
            step.orientation=world_quat.quaternion;

            list.footstep_data_list.push_back(step);

        }
    }
    else
    {
        for (int i = 0; i < pelvis_valuesR.size(); ++i) {

            try{
                tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF, ros::Time(0), ros::Duration(3.0));
                tf_listener_.transformVector(VAL_COMMON_NAMES::WORLD_TF,pelvis_valuesR[i],world_values);
                tf_listener_.transformQuaternion(VAL_COMMON_NAMES::WORLD_TF,pelvis_quatR[i],world_quat);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                return false;
            }

            step.robot_side=i%2 ? LEFT : RIGHT;
            step.location=world_values.vector;
            step.orientation=world_quat.quaternion;

            list.footstep_data_list.push_back(step);

        }
    }

    // publish footsteps

    this->footsteps_pub_.publish(list);
    ValkyrieWalker::id--;
    this->waitForSteps(list.footstep_data_list.size());

}

// wait till all the steps are taken
void ValkyrieWalker::waitForSteps(int n)
{
    while (step_counter < n && ros::ok())
    {
        ros::spinOnce();

        // hack to detect if robot has fallen and to exit this block
        if ((ros::Time::now() - cbTime_) > ros::Duration(5))
        {
            ROS_INFO("robot fallen, exiting");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    // reset back the counter
    step_counter = 0;
    return;
}
