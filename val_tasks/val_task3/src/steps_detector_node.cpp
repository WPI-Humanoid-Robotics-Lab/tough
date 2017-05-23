#include "val_task3/steps_detector.h"
#include "val_task3/stair_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "steps_detector");
  ros::NodeHandle nh;
  stair_detector sd(nh);
  std::vector<double> coefficients;
  geometry_msgs::Point dirVector;
  geometry_msgs::Point stairLoc;
  uint numSideBarDetected;
  sd.findStair(stairLoc, numSideBarDetected);
  coefficients = sd.coefficients();
  dirVector = sd.dirVector();
  steps_detector s1(nh);
  ROS_INFO_STREAM("1" << std::endl);
  s1.getStepsPosition(coefficients, dirVector, stairLoc);
  ROS_INFO_STREAM("2" << std::endl);
  ros::Rate loop(1);
  while(ros::ok())
  {
      ros::spinOnce();
      loop.sleep();
  }
}
