#include "val_task2/val_solar_detection.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "solar_plane_detection");
  ros::NodeHandle nh;
  geometry_msgs::Pose rover_loc;

  // obtained from rover detetion code
  rover_loc.position.x = 3.50641;
  rover_loc.position.y = -1.05765;
  rover_loc.position.z = 0;
  rover_loc.orientation.x = 0;
  rover_loc.orientation.y = 0;
  rover_loc.orientation.z = -0.380194;
  rover_loc.orientation.w = 0.924907;

  plane obj(nh,rover_loc);

  ros::Rate loop(1);
  while(ros::ok()){
      std::vector<geometry_msgs::Pose> poses;
      if(obj.getDetections(poses))
          for (size_t i = 0; i < poses.size(); ++i){
              //ROS_INFO_STREAM("x : "<<poses[i].position.x<<"y : "<<poses[i].position.y<<"z : "<<poses[i].position.z);
              //ROS_INFO_STREAM("x : "<<poses[i].orientation.x<<"y : "<<poses[i].orientation.y<<"z : "<<poses[i].orientation.z<<" w: "<<poses[i].orientation.w);
          }
      ros::spinOnce();
      loop.sleep();
  }
  return 0;
}
