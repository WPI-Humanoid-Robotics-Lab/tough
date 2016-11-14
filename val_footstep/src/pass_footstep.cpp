/*
 *
 *
 *
 * */

#include "val_footstep/pass_footstep.h"

enum FOOT{
    LEFT = 0,
    RIGHT = 1,
};


void stepsToVal::statCallback(const ihmc_msgs::FootstepStatusRosMessage & msg)
{

    if(msg.status == 1)
  {
       step_counter++;

       if (step_counter == 2 )
       {
           begin = ros::Time::now();
           std::cout << "start" << std::endl;
       }

       if (step_counter == 3 )
       {
           end = ros::Time::now();
          std::cout << "End" << std::endl;
       }

   }

 return;
}

 void stepsToVal::getFootstep(double startx,double starty,double goalx,double goaly , double startTh,double goalTh,ihmc_msgs::FootstepDataListRosMessage &list)
 {
      geometry_msgs::Pose2D start, goal;
       humanoid_nav_msgs::PlanFootsteps srv;

      start.x = startx;
      start.y =starty ;
      start.theta = startTh;
      goal.x = goalx ;
      goal.y = goaly;
      goal.theta = goalTh;

      srv.request.start = start;
      srv.request.goal = goal;

      if(this->footStep_client.call(srv))
      {
         for(int i =0; i <srv.response.footsteps.size();i++)
         {
         // srv.response.footsteps[i].x = ;

      }
      }


 }
stepsToVal::stepsToVal()
{
    this->footStep_client = n.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("pass_footsteps");
    this->footStepsToVal = n.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
    this->footStepStatus = n.subscribe("/ihmc_ros/valkyrie/output/footstep_status", 20,&stepsToVal::statCallback, this);

    tf_listener = new tf2_ros::TransformListener(this->tfBuffer);


    ros::Duration(0.5).sleep();
    step_counter = 0;
    std::string robot_name,right_foot_frame,left_foot_frame;

    if (n.getParam("/ihmc_ros/robot_name",robot_name))
    {
        if(n.getParam("/ihmc_ros/valkyrie/right_foot_frame_name", right_foot_frame) && n.getParam("/ihmc_ros/valkyrie/left_foot_frame_name", left_foot_frame))
        {
            Right_Foot_Frame.data = right_foot_frame;
            Left_Foot_Frame.data = left_foot_frame;
            ROS_INFO("%s", Right_Foot_Frame.data.c_str());
            ROS_INFO("%s", Left_Foot_Frame.data.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to get param 'my_param'");
    }
}

void stepsToVal::walk()
{
    ihmc_msgs::FootstepDataListRosMessage list ;
    list.transfer_time = 1;
    list.swing_time = 1;
    list.execution_mode = 0;
    list.unique_id = -1 ;

    list.footstep_data_list.push_back(this->getOffsetStep(LEFT , 0.4));
    list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , 0.8));
    list.footstep_data_list.push_back(this->getOffsetStep(LEFT , 1.2));
    list.footstep_data_list.push_back(this->getOffsetStep(RIGHT , 1.6));

    ros::Rate loop_rate(10);

    this->footStepsToVal.publish(list);

    ROS_INFO("Published data ");

   this->waitForSteps(list.footstep_data_list.size());


   return;
}

void stepsToVal::getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage & foot)
{

    std_msgs::String foot_frame;
    if (side == LEFT)
    {
        foot_frame = this->Left_Foot_Frame;


    }
    else
    {
        foot_frame = this->Right_Foot_Frame;

    }

    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tfBuffer.lookupTransform( "world",foot_frame.data,ros::Time(0));
    foot.orientation = transformStamped.transform.rotation;
    foot.location = transformStamped.transform.translation;
    foot.robot_side = side;
    foot.trajectory_type = 0;
    return;
}

ihmc_msgs::FootstepDataRosMessage stepsToVal::getOffsetStep(int side , double x)
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

void stepsToVal::waitForSteps(int n)
{
    while (step_counter < n)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();


    }
    return;
}

int main(int argc, char **argv)

{

    ros::init(argc, argv, "pass_footstep");




        stepsToVal agent;

        agent.walk();
       // agent.end = ros::Time::now();


        std::cout << " time taken  " << agent.end - agent.begin << std::endl;

    return 0;
}
