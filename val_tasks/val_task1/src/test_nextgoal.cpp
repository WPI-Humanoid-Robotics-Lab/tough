#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <val_common/val_common_names.h>
#include <val_task1/val_task1_utils.h>


int main(int argc, char** argv){

  ros::init(argc, argv, "test_next_goal");
  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);

  task1Utils utils(nh);

  ros::Rate loop_rate(0.5);

  geometry_msgs::Pose2D pose2D;
  int id = 0;

  while(ros::ok())
  {
      utils.getNextPoseToWalk(pose2D);

      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.id = id++;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::ARROW;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = pose2D.x;
      marker.pose.position.y = pose2D.y;
      marker.pose.position.z = 0;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose2D.theta), marker.pose.orientation);

      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      pub.publish(marker);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}

