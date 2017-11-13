#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/val_common_names.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_neck_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the neck");

  ros::Publisher log_pub = nh.advertise<std_msgs::String>(VAL_COMMON_NAMES::LOG_TOPIC, 10);
  const auto log_msg = [&log_pub](const std::string &str) {
      std_msgs::String msg;
      msg.data = ros::this_node::getName() + ": " + str;
      log_pub.publish(msg);
      ROS_INFO("%s", msg.data.c_str());
  };

  // wait a reasonable amount of time for the subscriber to connect
  ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
  while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until) {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }


  HeadTrajectory headTraj(nh);

  log_msg("Moving neck joints to 0, 0, 0");
  headTraj.moveNeckJoints({{ 0.0f, 0.0f, 0.0f }}, 2.0f);

  ros::spinOnce();
  ros::Duration(2).sleep();

  log_msg("Motion finished");
  return 0;
}
