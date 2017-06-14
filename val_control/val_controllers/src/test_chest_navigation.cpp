#include <val_controllers/val_chest_navigation.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <val_common/val_common_names.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_chest_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the chest");

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

  chestTrajectory chestTraj(nh);

  if(argc != 4) {
    chestTraj.controlChest(10, 0, 0);
    log_msg("Moving chest angle to 10, 0, 0");
  } else {
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);

    log_msg("Moving chest angle to " + std::to_string(roll) + ", " + std::to_string(pitch) + ", " + std::to_string(yaw));
    chestTraj.controlChest(roll, pitch, yaw);
  }

  ros::spinOnce();
  ros::Duration(2).sleep();

  log_msg("Motion finished");
  return 0;
}
