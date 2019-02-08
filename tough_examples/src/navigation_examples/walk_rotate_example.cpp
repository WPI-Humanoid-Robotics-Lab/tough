#include <iostream>
#include <tough_footstep/RobotWalker.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include "geometry_msgs/Pose2D.h"
#include "tough_common/robot_state.h"

using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "walk_rotate");
  ros::NodeHandle nh;

  ros::Publisher log_pub = nh.advertise<std_msgs::String>(TOUGH_COMMON_NAMES::LOG_TOPIC, 10);
  const auto log_msg = [&log_pub](const std::string& str) {
    std_msgs::String msg;
    msg.data = ros::this_node::getName() + ": " + str;
    log_pub.publish(msg);
    ROS_INFO("%s", msg.data.c_str());
  };

  // wait a reasonable amount of time for the subscriber to connect
  ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
  while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  RobotWalker walk(nh, 1.0, 1.0, 0);
  geometry_msgs::Pose2D goal;

  ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
  walk.getCurrentStep(RIGHT, *current);

  // get current position
  goal.x = current->location.x;
  goal.y = current->location.y;
  if (argc == 2)
  {
    goal.theta = tf::getYaw(current->orientation) + std::atof(argv[1]);
    log_msg("Rotating " + std::to_string(std::atof(argv[1])) + " (to yaw " + std::to_string(goal.theta) + " in world "
                                                                                                          "frame)");
    walk.walkToGoal(goal);
  }
  else
  {
    log_msg("Expected 1 argument, but got " + std::to_string(argc - 1) + ". Exiting.");
  }

  log_msg("Rotation complete");

  return 0;
}
