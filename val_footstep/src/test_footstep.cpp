
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

int main(int argc, char **argv)
{
 ros::init(argc, argv, "test_footstep");
 ros::NodeHandle n;
 ros::ServiceClient footStep_client = n.serviceClient <val_footstep::footStep> ("footStepService"); // Creating client for footstep service

 ros::Publisher footStep_pub = n.advertise<val_footstep::StepTargetArray>("footStepPub", 50,true);

val_footstep::footStep srv;

geometry_msgs::Pose2D start, goal;
start.x = 1.5;
start.y = 0.5 ;
start.theta = 0;
goal.x = 2.5 ;
goal.y = 2.5 ;
goal.theta = 0;
srv.request.start = start;
srv.request.goal = goal;
val_footstep::StepTargetArray steps;
 ROS_INFO("Calling footstep planner ");
if(footStep_client.call(srv)) // Service Call
{

   ROS_INFO("%d", (int)srv.response.result.steps.size());
  steps.steps = srv.response.result.steps;
  footStep_pub.publish(steps);
   ROS_INFO("Footstep Ready ");

}
ros::spin();
return 0;
}
