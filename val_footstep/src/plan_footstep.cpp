#include "ros/ros.h"
#include <cstdlib>
#include"geometry_msgs/PoseStamped.h"
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include"geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <val_footstep/StepTargetArray.h>
#include "std_msgs/Float64MultiArray.h"
#include <boost/array.hpp>
#include<val_footstep/footStep.h>


bool planSteps(val_footstep::footStep::Request  &req, val_footstep::footStep::Response &res)
{

    ros::NodeHandle p;
    ros::ServiceClient footStep_client = p.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("plan_footsteps"); // Calling footstep_planner_node
    humanoid_nav_msgs::PlanFootsteps srv;
    srv.request.start = req.start;
    srv.request.goal = req.goal;
    ROS_INFO("Calling footstep planner ");

    if(footStep_client.call(srv))
    {
        ROS_INFO("No of FootSteps : [%d]" , (int)srv.response.footsteps.size());


        res.result.steps = srv.response.footsteps;
        ROS_INFO("Footstep Ready ");


    }

     return true;
}










int main(int argc, char **argv)

{
    ros::init(argc, argv, "plan_footstep");
    ros::NodeHandle n;


    ros::ServiceServer footStepService = n.advertiseService("footStepService", planSteps); // Service Created for Getting footsteps
    ros::spin();



    return 0;
}
