#include <tough_controller_interface/pelvis_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/val_common_names.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pelvis_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the pelvis");

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

  PelvisControlInterface pelvisTraj(nh);

  if(argc != 2) {
    log_msg("making pelvis do a dance");
    pelvisTraj.controlPelvisHeight(1.25f);
    ros::Duration(2).sleep();
    pelvisTraj.controlPelvisHeight(0.9f);
    ros::Duration(2).sleep();
    pelvisTraj.controlPelvisHeight(1.0f);
  }
  else {
    float height = std::atof(argv[1]);
    log_msg("moving pelvis to height " + std::to_string(height));
    pelvisTraj.controlPelvisHeight(height);
  }


  ros::spinOnce();
  ros::Duration(2).sleep();

  log_msg("motion complete");

  return 0;
}
