#include "val_task2/val_solar_detection.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "solar_plane_detection");
  ros::NodeHandle nh;
  geometry_msgs::Pose rover_loc;

  // obtained from rover detetion code
  rover_loc.position.x = 3.34137;
  rover_loc.position.y = 1.17478;
  rover_loc.position.z = 0;
  rover_loc.orientation.x = 0;
  rover_loc.orientation.y = 0;
  rover_loc.orientation.z = 0.379036;
  rover_loc.orientation.w = 0.925382;

  plane obj(nh,rover_loc);

  while(ros::ok()){

    ros::spinOnce();
  }
  return 0;
}
