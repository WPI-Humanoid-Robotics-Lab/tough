/*
 *
 *
 *
 * */

#include "val_footstep/pass_footstep.h"
#include <ros/macros.h>
/// \todo This can go in val_common
enum FOOT{
    LEFT = 0,
    RIGHT = 1,
};


void ValkyrieWalker::footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage & msg)
{
    if(msg.status == 1)
    {
        step_counter++;
        ROS_INFO("step counter : %d",step_counter);

    }

    return;
}

void ValkyrieWalker::getFootstep(double goalx,double goaly ,double goalTh,ihmc_msgs::FootstepDataListRosMessage &list)
{
    /// \todo fix the robot pose, if the legs are not together before walking.

    geometry_msgs::Pose2D start, goal;
    humanoid_nav_msgs::PlanFootsteps srv;

    ihmc_msgs::FootstepDataRosMessage* startstep = new ihmc_msgs::FootstepDataRosMessage() ;
    this->getCurrentStep(0,*startstep);

    /// \todo get start from robot position
    start.x = startstep->location.x ;//-  0.075356;
    start.y = startstep->location.y;// - 0.0558756;
    start.theta = tf::getYaw(startstep->orientation);
    /*
      std::cout<< "Left leg x  "<<startstep->location.x <<  std::endl;
      std::cout<< "Left leg y "<<startstep->location.y <<std::endl;

      this->getCurrentStep(1,*startstep);
      start.x += startstep->location.x;
      start.y += startstep->location.y;
      start.theta += tf::getYaw(startstep->orientation);
      start.x /=2;
      start.y /=2;
      start.theta /=2;
      std::cout<< "Right leg x "<<startstep->location.x <<std::endl;
      std::cout<< "Right leg y "<< startstep->location.y<<std::endl

      start.x =0.0;
      start.y =0.0;
      start.theta =0.0;
      */
    delete startstep;
    goal.x = goalx ;
    goal.y = goaly;
    goal.theta = goalTh;

    srv.request.start = start;
    srv.request.goal = goal;

    //std::cout<<"Calling service"<<std::endl;

    if(this->footstep_client.call(srv))
    {

        //   std::cout<< "Inside Service "<< std::endl;

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
ValkyrieWalker::ValkyrieWalker(ros::NodeHandle nh):n(nh)
{
    this->footstep_client = n.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("plan_footsteps");
    this->footsteps_to_val = n.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
    this->footstep_status = n.subscribe("/ihmc_ros/valkyrie/output/footstep_status", 20,&ValkyrieWalker::footstepStatusCB, this);

    tf_listener = new tf2_ros::TransformListener(this->tfBuffer);

    //Fill in the tf buffer for 0.5 sec
    //wait for tf
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

ValkyrieWalker::~ValkyrieWalker(){
    delete tf_listener;
}

void ValkyrieWalker::walk()
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1.0;
    list.swing_time = 1.0;
    list.execution_mode = 0;
    list.unique_id = -1 ;

    list.footstep_data_list.push_back(this->getOffsetStep(LEFT , 0.6));
    list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , 1.2));
    list.footstep_data_list.push_back(this->getOffsetStep(LEFT , 1.8));
    list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , 2.4));
    list.footstep_data_list.push_back(this->getOffsetStep(LEFT , 3.0));
    list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , 3.0));
    //this->getFootstep(2.0,0.0,0.0,list);
    ros::Rate loop_rate(10);

    this->footsteps_to_val.publish(list);

    ROS_INFO("Published data on topic");
    std::cout<<"no of steps  " << list.footstep_data_list.size()<<std::endl;
    this->waitForSteps(list.footstep_data_list.size());

    ihmc_msgs::FootstepDataRosMessage* startstep = new ihmc_msgs::FootstepDataRosMessage() ;
    this->getCurrentStep(0,*startstep);
    std::cout<< "Left leg x  "<<startstep->location.x <<  std::endl;
    std::cout<< "Left leg y "<<startstep->location.y <<std::endl;
    delete startstep;
    return;
}

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
    transformStamped = tfBuffer.lookupTransform( "world",foot_frame.data,ros::Time(0));
    foot.orientation = transformStamped.transform.rotation;
    foot.location = transformStamped.transform.translation;
    foot.robot_side = side;
    foot.trajectory_type = 0;
    return;
}

ihmc_msgs::FootstepDataRosMessage ValkyrieWalker::getOffsetStep(int side , double x)
{

    ihmc_msgs::FootstepDataRosMessage * next = new ihmc_msgs::FootstepDataRosMessage();

    this->getCurrentStep(side, *next);
    next->location.x+=x;
    /*
    std::cout<< " robot side = " <<     next->robot_side << std::endl;
    std::cout<< " orientation data w = " << next->orientation.w << std::endl;
    std::cout<< " orientation data x = " << next->orientation.x << std::endl;
    std::cout<< " orientation data y = " << next->orientation.y << std::endl;
    std::cout<< " orientation data z = " << next->orientation.z << std::endl;
    std::cout<< " location data x = " << next->location.x << std::endl;
    std::cout<< " location data y = " << next->location.y << std::endl;
    std::cout<< " location data z = " << next->location.z << std::endl;
    */
    return (*next);


}

void ValkyrieWalker::waitForSteps(int n)
{
    while (step_counter <n)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
         std::cout<< " inside wait for steps function "<< std::endl;


    }
    return;
}

int main(int argc, char **argv)

{

    ros::init(argc, argv, "pass_footstep");
    ros::NodeHandle nh;

    ValkyrieWalker agent(nh);


    agent.walk();
    // agent.end = ros::Time::now();




    return 0;
}
