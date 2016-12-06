/*
 *
 *
 *
 * */

#include "val_footstep/ValkyrieWalker.h"
#include <iostream>


// CallBack function for walking status
///\todo Must have more status feedback from the Robot. Should know if it did not complete the step then what happened.
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
    list.transfer_time = transfer_time;
    list.swing_time = swing_time;
    list.execution_mode = exe_mode;
    list.unique_id = ValkyrieWalker::id;


    if(this->getFootstep(goal,list))
    {
        this->footsteps_to_val.publish(list);

        ROS_INFO("Published data on topic");
        ValkyrieWalker::id--;
    }
    this->waitForSteps(list.footstep_data_list.size());


    return true;

}

// creates and n footsteps of width step_size
bool ValkyrieWalker::WalkNStepsForward(int n, float x_offset, float y_offset, bool continous)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = transfer_time;
    list.swing_time = swing_time;
    list.execution_mode = exe_mode;

    list.unique_id = ValkyrieWalker::id ;

    for (int m =1; m <= n ; m++)
    {
        if(m%2 == 1)
            list.footstep_data_list.push_back(*getOffsetStep(LEFT , m*x_offset, m*y_offset));
        else
            list.footstep_data_list.push_back(*getOffsetStep(RIGHT , m*x_offset, m*y_offset));
    }

    if(!continous){
        if (n%2 ==1)
            list.footstep_data_list.push_back(*getOffsetStep(RIGHT , n*x_offset, n*y_offset));
        if (n%2 ==0)
            list.footstep_data_list.push_back(*getOffsetStep(LEFT , n*x_offset, n*y_offset));
    }

    /*
    this->footsteps_to_val.publish(list);
   
    ValkyrieWalker::id--;
    this->waitForSteps(list.footstep_data_list.size());
   */
    this->WalkGivenSteps(list);
    return true;
}

//creates and n footsteps of width step_size backwards
bool   ValkyrieWalker::WalkNStepsBackward(int n, float x_offset, float y_offset, bool continous)
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = transfer_time;
    list.swing_time = swing_time;
    list.execution_mode = exe_mode;
    list.unique_id = ValkyrieWalker::id;

    

    for (int m =1; m <= n ; m++)
    {
        if(m%2 == 1)
            list.footstep_data_list.push_back(*getOffsetStep(LEFT , -m*x_offset, -m*y_offset));
        else
            list.footstep_data_list.push_back(*getOffsetStep(RIGHT , -m*x_offset, -m*y_offset));
    }

    if(!continous) {
            if (n%2 ==1)
            list.footstep_data_list.push_back(*getOffsetStep(RIGHT , -n*x_offset, n*y_offset));
        if (n%2 ==0)
            list.footstep_data_list.push_back(*getOffsetStep(LEFT , -n*x_offset, n*y_offset));

    }


    /*std::cout<< " size of array " << list.footstep_data_list.size() << std::endl; 
    this->footsteps_to_val.publish(list);
    ValkyrieWalker::id--;
    this->waitForSteps(list.footstep_data_list.size());
    */
    this->WalkGivenSteps(list);
    return true;

}


bool ValkyrieWalker::WalkGivenSteps(ihmc_msgs::FootstepDataListRosMessage& list )
{
    this->footsteps_to_val.publish(list);
    ValkyrieWalker::id--;
    this->waitForSteps(list.footstep_data_list.size());
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
    start.y = startstep->location.y - 0.12;
    std::cout<< "Start Position  x = " << start.x << "  y = " << start.y<<std::endl;

    start.theta = tf::getYaw(startstep->orientation);

    delete startstep;


    srv.request.start = start;
    srv.request.goal = goal;



    if(this->footstep_client.call(srv))
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
            ROS_INFO("Step x  %d %.2f", i, srv.response.footsteps.at(i).pose.x);
            ROS_INFO("Step y  %d %.2f", i, srv.response.footsteps.at(i).pose.y);
            ROS_INFO("Side  %d %d",i, int(side));

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
void ValkyrieWalker::setWalkParms(float InTransferTime,float InSwingTime, int InMode)
{
    this->transfer_time = InTransferTime;
    this->swing_time = InSwingTime;
    this->exe_mode = InMode;
}

// constructor

ValkyrieWalker::ValkyrieWalker(ros::NodeHandle nh,double InTransferTime ,double InSwingTime, int InMode):n(nh)
{
    this->footstep_client = n.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("plan_footsteps");
    this->footsteps_to_val = n.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
    this->footstep_status = n.subscribe("/ihmc_ros/valkyrie/output/footstep_status", 20,&ValkyrieWalker::footstepStatusCB, this);

    transfer_time = InTransferTime;
    swing_time = InSwingTime;
    exe_mode = InMode;

    tf_listener = new tf2_ros::TransformListener(this->tfBuffer);

    ros::Duration(0.5).sleep();
    step_counter = 0;
    ValkyrieWalker::id = -1;
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

ihmc_msgs::FootstepDataRosMessage* ValkyrieWalker::getOffsetStep(int side , float x, float y)
{
    ///\ todo shared pointer implementation
    ihmc_msgs::FootstepDataRosMessage *next = new ihmc_msgs::FootstepDataRosMessage();

    this->getCurrentStep(side, *next);
    next->location.x+=x;
    next->location.y+=y;

    return next;

}
// wait till all the steps are taken
void ValkyrieWalker::waitForSteps(int n)
{
    while (step_counter <n && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return;
}

//int main(int argc, char **argv)

//{

//    ros::init(argc, argv, "pass_footstep");
//    ros::NodeHandle nh;
//    ValkyrieWalker agent(nh,1,1,0);



//    while(ros::ok()) {
//        geometry_msgs::Pose2D goal;
//        float x = 0.0;
//        float y= 0.0;
//        float theta = 0.0;
//        int flag ;
////        std::cout<<"Enter x coordinate of goal : ";
////        std::cin>>x;
////        std::cout<<"Enter y coordinate of goal : ";
////        std::cin>>y;
////        std::cout<<"Enter angle of rotation for goal in radians : ";
////        std::cin>>theta;

////        agent.WalkNStepsForward(2,0.4,-0.1, true);
////        agent.WalkNStepsForward(2,0.4,0, true);
////        agent.WalkNStepsForward(1,0.4,0);

//        goal.x = x;
//        goal.y = y;
//        goal.theta = theta;



////        // agent.WalkNStepsBackward(2);

//        agent.WalkToGoal(goal);

//        std::cout<<"Enter 0 to exit or 1 to continue \n";
//        std::cin>>flag;

//        if(!flag)
//            break;
//    }

//    return 0;
//}
