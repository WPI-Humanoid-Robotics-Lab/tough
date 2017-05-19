#include "val_task3/steps_detector.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "steps_detector");
  ros::NodeHandle nh;
  steps_detector s1(nh);
  ros::Rate loop(1);
  while(ros::ok())
  {
      ros::spinOnce();
      loop.sleep();
  }
}
