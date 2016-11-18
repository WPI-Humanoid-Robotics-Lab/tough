#include <footstep_planner/FootstepPlannerNode.h>
#include "ros/ros.h"

#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
 ros::init(argc, argv, "footstep_node");

  footstep_planner::FootstepPlannerNode planner;
   ros::spin();

 return 0;


}
