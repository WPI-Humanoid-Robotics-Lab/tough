#include "val_task2/val_solar_panel_detector.h"
#include "val_task2/val_rover_detection.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "solar_plane_detection");
  ros::NodeHandle nh;
  geometry_msgs::Pose2D rover_loc;
  std::vector<geometry_msgs::Pose> rover_poses, rover_poses_fine;
  bool isroverRight;

  ros::Rate loop1(1);

  if(true) //to destruct the rover_obj
  {
      RoverDetector rover_obj(nh);
      RoverDetector rover_obj2(nh,true);
      while(ros::ok())
      {

          rover_obj.getDetections(rover_poses);
          rover_obj2.getDetections(rover_poses_fine);
          if(!(rover_poses.empty() || rover_poses_fine.empty()))
          {
              geometry_msgs::Pose rover_loc_3d;
              rover_loc_3d = rover_poses_fine[rover_poses_fine.size()-1];
              rover_loc.x = rover_loc_3d.position.x;
              rover_loc.y = rover_loc_3d.position.y;
              rover_loc.theta = tf::getYaw(rover_poses.back().orientation);
              ROVER_SIDE roverSide;
              if(rover_obj.getRoverSide(roverSide)){
                  isroverRight = roverSide == ROVER_SIDE::RIGHT;
                  break;
              }
          }
          ros::spinOnce();
          loop1.sleep();
      }
  }

  SolarPanelDetect obj(nh,rover_loc,isroverRight);

  ros::Rate loop(1);
  while(ros::ok()){
      std::vector<geometry_msgs::Pose> poses;
      if(obj.getDetections(poses))
          for (size_t i = 0; i < poses.size(); ++i){

              RobotStateInformer* robot_state = RobotStateInformer::getRobotStateInformer(nh);
              geometry_msgs::Pose poseInPelvisFrame;

              robot_state->transformPose(poses[i], poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);

              tfScalar r, p, y;
              tf::Quaternion q;
              tf::quaternionMsgToTF(poseInPelvisFrame.orientation, q);
              tf::Matrix3x3 rot(q);
              rot.getRPY(r, p, y);
              ROS_INFO("=========================================================");
              ROS_INFO("Roll %.2f, Pitch %.2f, Yaw %.2f", (float)r, (float)p, (float)y);


              ROS_INFO_STREAM("x : "<<poseInPelvisFrame.position.x<<"y : "<<poseInPelvisFrame.position.y<<"z : "<<poseInPelvisFrame.position.z);
              ROS_INFO_STREAM("x y z w : "<<poseInPelvisFrame.orientation.x<<" "<<poseInPelvisFrame.orientation.y<<" "<<poseInPelvisFrame.orientation.z<<" "<<poseInPelvisFrame.orientation.w);
              obj.invertYaw(poseInPelvisFrame);
              ROS_INFO("outward Orientation");
              ROS_INFO_STREAM("x : "<<poseInPelvisFrame.position.x<<"y : "<<poseInPelvisFrame.position.y<<"z : "<<poseInPelvisFrame.position.z);
              ROS_INFO_STREAM("x y z w : "<<poseInPelvisFrame.orientation.x<<" "<<poseInPelvisFrame.orientation.y<<" "<<poseInPelvisFrame.orientation.z<<" "<<poseInPelvisFrame.orientation.w);
              ROS_INFO("=========================================================");
          }
      ros::spinOnce();
      loop.sleep();
  }
  return 0;
}
