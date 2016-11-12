#ifndef PASS_FOOTSTEP_HPP
#define PASS_FOOTSTEP_HPP

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
#include"geometry_msgs/Vector3.h"
#include"geometry_msgs/Quaternion.h"
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include <tf2_ros/transform_listener.h>
#include"tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"


class stepsToVal
 {
   
 public:
 stepsToVal();

 tf2_ros::Buffer tfBuffer;
 tf2_ros::TransformListener* tf_listener;

  std_msgs::String Right_Foot_Frame,Left_Foot_Frame;
  int step_counter;
 void walk();
 void getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage& foot);
 void statCallback(const ihmc_msgs::FootstepStatusRosMessage & msg);
 ihmc_msgs::FootstepDataRosMessage getOffsetStep(int side, double x);
 void waitForSteps( int n); 
 private:
   
   ros::NodeHandle n;
   ros::ServiceClient footStep_client ;
   ros::Publisher footStepsToVal ;
   ros::Subscriber footStepStatus ;  
 };


#endif
