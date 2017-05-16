#include "val_task2/val_solar_detection.h"
#include "val_task2/val_rover_detection.h"
int main(int argc, char** argv){
  ros::init(argc, argv, "solar_array_detection");
  ros::NodeHandle nh;
  geometry_msgs::Pose2D rover_loc;
  std::vector<geometry_msgs::Pose> rover_poses;
  bool isroverRight;

  ros::Rate loop1(1);

  if(true) //to destruct the rover_obj
  {
      RoverDetector rover_obj(nh);
      while(ros::ok())
      {

          rover_obj.getDetections(rover_poses);
          if(rover_poses.size())
          {
              geometry_msgs::Pose rover_loc_3d;
              rover_loc_3d = rover_poses[rover_poses.size()-1];
              rover_loc.x = rover_loc_3d.position.x;
              rover_loc.y = rover_loc_3d.position.y;
              rover_loc.theta = tf::getYaw(rover_loc_3d.orientation);
              isroverRight = rover_obj.isRoverOnRight();
              break;
          }
          ros::spinOnce();
          loop1.sleep();
      }
  }


  RoverBlocker obj(nh,rover_loc,isroverRight);

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
