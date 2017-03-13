/*
 *
 *
 *
 * */

#include "val_footstep/ValkyrieWalker.h"
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

bool ValkyrieWalker::walkToGoal( geometry_msgs::Pose2D &goal)
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
        this->waitForSteps(list.footstep_data_list.size());
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

    tf_listener_ = new tf2_ros::TransformListener(this->tf_buffer_);

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
    delete tf_listener_;
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

    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tf_buffer_.lookupTransform( "world",foot_frame.data,ros::Time(0),ros::Duration(10.0));
    foot.orientation = transformStamped.transform.rotation;
    foot.location = transformStamped.transform.translation;
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

//int main(int argc, char **argv)

//{
//    ros::init(argc, argv, "test_walking");
//    ros::NodeHandle nh;
//    ValkyrieWalker agent(nh,1,1,0);
//    float transferTime, swingTime, swingHeight, stepLength;
//    if ( argc != 5 ) // argc should be 2 for correct execution
//        std::cout<<"usage: "<< argv[0] <<" transferTime swingTime swingHeight stepLengtho\n";
//    else {
//        transferTime = std::atof(argv[1]);
//        swingTime = std::atof(argv[2]);
//        swingHeight = std::atof(argv[3]);
//        stepLength = std::atof(argv[4]);
//    }

//    agent.setWalkParms(transferTime, swingTime, 0);
//    agent.setSwing_height(swingHeight);
//    agent.WalkNSteps(5,stepLength);

//    return 0;
//}
