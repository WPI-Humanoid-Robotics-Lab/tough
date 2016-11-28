/*
 *
 *
 *
 * */

#include "val_footstep/ValkyrieWalker.h"
#include <ros/macros.h>
/// \todo This can go in val_common

// Defining foot
enum FOOT{
    LEFT = 0,
    RIGHT = 1,
};

// CallBack function for walking status
void ValkyrieWalker::footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage & msg)
{
    if(msg.status == 1)
    {
        step_counter++;
        ROS_INFO("step counter : %d",step_counter);

    }

    return;
}

// creates and send footsteps to  val to reach goal position

bool ValkyrieWalker::WalkToGoal( geometry_msgs::Pose2D &goal)
{

    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1.0;
    list.swing_time = 1.0;
    list.execution_mode = 0;
    list.unique_id = -1 ;


    this->getFootstep(goal,list);
    this->footsteps_to_val.publish(list);

    ROS_INFO("Published data on topic");

    this->waitForSteps(list.footstep_data_list.size());


    return true;

}

// creates and n footsteps of width step_size
bool ValkyrieWalker::WalkNStepsForward(int n, float step_size)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1.0;
    list.swing_time = 1.0;
    list.execution_mode = 0;
    list.unique_id = -1 ;


    for (int m =1; m <= n ; m++)
    {
       if(m%2 == 1)
        list.footstep_data_list.push_back(this->getOffsetStep(LEFT , m*step_size));
       else
         list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , m*step_size));
    }

    if (n%2 ==1)
  { list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , n*step_size));}
    if (n%2 ==0)
   { list.footstep_data_list.push_back(this->getOffsetStep(LEFT , n*step_size));}


    this->footsteps_to_val.publish(list);
    this->waitForSteps(list.footstep_data_list.size());



    return true;
}

//creates and n footsteps of width step_size backwards
bool   ValkyrieWalker::WalkNStepsBackward(int n, float step_size)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1.0;
    list.swing_time = 1.0;
    list.execution_mode = 0;
    list.unique_id = -1 ;


    for (int m =1; m <= n ; m++)
    {
       if(m%2 == 1)
        list.footstep_data_list.push_back(this->getOffsetStep(LEFT , -m*step_size));
       else
         list.footstep_data_list.push_back(this->getOffsetStep(RIGHT ,- m*step_size));
    }

    if (n%2 ==1)
  { list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , -n*step_size));}
    if (n%2 ==0)
   { list.footstep_data_list.push_back(this->getOffsetStep(LEFT , -n*step_size));}

    this->footsteps_to_val.publish(list);
    this->waitForSteps(list.footstep_data_list.size());
    return true;

}


//Calls the footstep planner service to get footsteps to reach goal

void ValkyrieWalker::getFootstep(geometry_msgs::Pose2D &goal,ihmc_msgs::FootstepDataListRosMessage &list)
{
    /// \todo fix the robot pose, if the legs are not together before walking.

    geometry_msgs::Pose2D start;
    humanoid_nav_msgs::PlanFootsteps srv;

    ihmc_msgs::FootstepDataRosMessage* startstep = new ihmc_msgs::FootstepDataRosMessage() ;
    this->getCurrentStep(0,*startstep);

    /// \todo get start from robot position
    start.x = startstep->location.x ;
    start.y = startstep->location.y;
    start.theta = tf::getYaw(startstep->orientation);

    delete startstep;


    srv.request.start = start;
    srv.request.goal = goal;



    if(this->footstep_client.call(srv))
    {


        for(int i=0; i <srv.response.footsteps.size();i++)
        {


            ihmc_msgs::FootstepDataRosMessage* step = new ihmc_msgs::FootstepDataRosMessage() ;
            bool side = bool(srv.response.footsteps.at(i).leg);

            side = !side;

            this->getCurrentStep(int(side),*step);

            step->location.x = srv.response.footsteps.at(i).pose.x;
            step->location.y = srv.response.footsteps.at(i).pose.y;


            tf::Quaternion t = tf::createQuaternionFromYaw(srv.response.footsteps.at(i).pose.theta);
            ROS_INFO("Step x  %d %.2f", i, srv.response.footsteps.at(i).pose.x);
            ROS_INFO("Step y  %d %.2f", i, srv.response.footsteps.at(i).pose.y);
            ROS_INFO("Side  %d %d",i, int(side));

            step->orientation.w = t.w();
            step->orientation.x = t.x();
            step->orientation.y = t.y();
            step->orientation.z = t.z();

            list.footstep_data_list.push_back(*step);
        }
    }


}

// constructor

ValkyrieWalker::ValkyrieWalker(ros::NodeHandle nh):n(nh)
{
    this->footstep_client = n.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("plan_footsteps");
    this->footsteps_to_val = n.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
    this->footstep_status = n.subscribe("/ihmc_ros/valkyrie/output/footstep_status", 20,&ValkyrieWalker::footstepStatusCB, this);

    tf_listener = new tf2_ros::TransformListener(this->tfBuffer);

     ros::Duration(0.5).sleep();
    step_counter = 0;
    std::string robot_name;

    if (n.getParam("/ihmc_ros/robot_name",robot_name))
    {
        if(n.getParam("/ihmc_ros/valkyrie/right_foot_frame_name", right_foot_frame.data) && n.getParam("/ihmc_ros/valkyrie/left_foot_frame_name", left_foot_frame.data))
        {
            ROS_DEBUG("%s", right_foot_frame.data.c_str());
            ROS_DEBUG("%s", left_foot_frame.data.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to get param foot frames.");
    }
}
// Destructor
ValkyrieWalker::~ValkyrieWalker(){
    delete tf_listener;
}

// Get starting location of the foot

void ValkyrieWalker::getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage & foot)
{

    std_msgs::String foot_frame;
    if (side == LEFT)
    {
        foot_frame = this->left_foot_frame;


    }
    else
    {
        foot_frame = this->right_foot_frame;

    }

    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tfBuffer.lookupTransform( "world",foot_frame.data,ros::Time(0),ros::Duration(10.0));
    foot.orientation = transformStamped.transform.rotation;
    foot.location = transformStamped.transform.translation;
    foot.robot_side = side;
    foot.trajectory_type = 0;
    return;
}

// gives footstep which are offset from current step (only for straight line)

ihmc_msgs::FootstepDataRosMessage ValkyrieWalker::getOffsetStep(int side , double x)
{

    ihmc_msgs::FootstepDataRosMessage * next = new ihmc_msgs::FootstepDataRosMessage();

    this->getCurrentStep(side, *next);
    next->location.x+=x;

    return (*next);


}
// wait till all the steps are taken
void ValkyrieWalker::waitForSteps(int n)
{
    while (step_counter <n)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();


    }
    return;
}

int main(int argc, char **argv)

{

    ros::init(argc, argv, "pass_footstep");
    ros::NodeHandle nh;

    geometry_msgs::Pose2D goal;

    goal.x = 1.0;
    goal.y = 0.0;
    goal.theta = 0;

    ValkyrieWalker agent(nh);

    //agent.WalkNStepsBackward(2,0.5);


    agent.WalkToGoal(goal);



    return 0;
}
