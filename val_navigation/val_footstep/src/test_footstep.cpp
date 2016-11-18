 #include <ros/ros.h>
 #include <humanoid_nav_msgs/PlanFootsteps.h>
 #include <val_footstep/StepTargetArray.h>


 int main(int argc, char **argv)

 {
  ros::init(argc, argv, "plan_footstep");
  ros::NodeHandle n;
  ros::ServiceClient footStep_client = n.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("plan_footsteps");
 // ros::Publisher footStep_publish = n.advertise < std::vector <humanoid_nav_msgs::StepTarget > >("footStep_pub", 50,true);
  //std::vector <humanoid_nav_msgs::StepTarget , 10 > a;

  //humanoid_nav_msgs::StepTarget p [5];

  ros::Publisher footStep_pub = n.advertise<val_footstep::StepTargetArray>("footStep_pub", 50,true);
  /*
 */

 humanoid_nav_msgs::PlanFootsteps srv;

 humanoid_nav_msgs::PlanFootsteps::Response d ;

 geometry_msgs::Pose2D start, goal;

 start.x = 0.0;
 start.y = 0 ;
 start.theta = 0;
 goal.x = 1.6 ;
 goal.y = 0 ;
 goal.theta = 0;

 srv.request.start = start;
 srv.request.goal = goal;

 val_footstep::StepTargetArray result;
  std::cout<< " calling" << std::endl;

 if(footStep_client.call(srv))
 {
    std::cout<< " size :" <<  srv.response.footsteps.size()<< std::endl;
   result.steps = srv.response.footsteps;

   footStep_pub.publish(result);
   std::cout<< " calling Done" << std::endl;
    //footStep_pub.publish(srv.response.result);
 //    ros::Rate loop_rate(10);
 //    while(ros::ok())
 //    {
 //        for(int m =0 ; m< srv.response.footsteps.size(); m++ )
 //        {
 //            footStep_pub.publish(srv.response.footsteps[m]);
 //        }

 //    }

 }
 while(ros::ok())
 {}




 return 0;
 }
