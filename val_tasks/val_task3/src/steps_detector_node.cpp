#include "val_task3/steps_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "steps_detector");
  ros::NodeHandle nh;
  steps_detector s1(nh);
  //geometry_msgs::Point stairLoc;
  //uint numSideBarsDetected;
  ros::Rate loop(1);
  //stair.findStair(stairLoc, numSideBarsDetected);
  //std::vector<double> coefficients =  stair.coefficients();
  //std::vector<Eigen::Vector4f> endPoints = stair.endPoints();
  //geometry_msgs::Point stairDir = stair.dirVector();

  while(ros::ok())
  {
      ros::spinOnce();
      loop.sleep();
  }
}
