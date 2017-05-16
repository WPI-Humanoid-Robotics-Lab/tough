#include "val_task2/val_solar_detection.h"
#include "val_task2/val_rover_detection.h"
int main(int argc, char** argv){
  ros::init(argc, argv, "solar_array_detection");
  ros::NodeHandle nh;
  geometry_msgs::Pose rover_loc;
  std::vector<geometry_msgs::Pose> rover_poses;

  ros::Rate loop1(1);

  if(true) //to destruct the rover_obj
  {
      RoverDetector rover_obj(nh);
      while(ros::ok())
      {

          rover_obj.getDetections(rover_poses);
          if(rover_poses.size())
          {
              rover_loc = rover_poses[rover_poses.size()-1];
              break;
          }
          ros::spinOnce();
          loop1.sleep();
      }
  }


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
