#include "val_task2/val_solar_detection.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "solar_plane_detection");
  ros::NodeHandle nh;
  geometry_msgs::Pose rover_loc;
  rover_loc.position.x = 1;
  rover_loc.position.y = 1;
  rover_loc.position.z = 1;
  rover_loc.orientation.x = 0;
  rover_loc.orientation.y = 0;
  rover_loc.orientation.z = 0;
  rover_loc.orientation.w = 1;

  plane obj(nh,rover_loc);

  while(ros::ok()){

    ros::spinOnce();
  }
  return 0;
}
