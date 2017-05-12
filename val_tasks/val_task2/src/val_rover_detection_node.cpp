#include "val_task2/val_rover_detection.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "rover_detection");

  ros::NodeHandle nh;

  rover obj(nh);

  while(ros::ok()){

    ros::spinOnce();
  }
  return 0;
}
