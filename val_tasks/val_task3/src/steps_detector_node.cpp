#include "val_task3/steps_detector.h"
#include "val_task3/stair_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "steps_detector");
  ros::NodeHandle nh;
  //stair_detector sd(nh);
  steps_detector s1(nh);
  bool foundSteps = false;
  int numIterations = 0;
  std::vector<double> coefficients;
  geometry_msgs::Point dirVector;
  geometry_msgs::Point stairLoc;
  uint numSideBarDetected;
  coefficients = { 0.0588899, -2.50763,	0, 2.91158};
  dirVector.x = 1;
  dirVector.y = 0.0234843;
  dirVector.z = 0;
  stairLoc.x = 3.56882;
  stairLoc.y = -1.07728;
  stairLoc.z = 0.758028;
  while(ros::ok())
  {
//      sd.findStair(stairLoc, numSideBarDetected);
//      coefficients = sd.coefficients();
//      dirVector = sd.dirVector();

//      ROS_INFO_STREAM("Coefficients : " << coefficients[0] << "\t" << coefficients[1] << "\t" << coefficients[2] << "\t" << coefficients[3] << std::endl);
//      ROS_INFO_STREAM("Direction Vector : " << dirVector << std::endl);
//      ROS_INFO_STREAM("Stair Loc : " << stairLoc << std::endl);
      foundSteps = s1.getStepsPosition(coefficients, dirVector, stairLoc);
      //ROS_INFO(foundStair ? "***** Steps detected" : "xxxxx Steps not detected");
      ros::spinOnce();
      numIterations++;
  }
}
