#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_nudgeLocal");
  ros::NodeHandle nh;

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


  armTrajectory armTraj(nh);
  ros::Duration(1).sleep();

  if(argc == 3){
    std::string side_str;
    armSide side;
    if(std::atoi(argv[1]) == 0){
      side = LEFT;
      side_str = "left";
    } else {
      side = RIGHT;
      side_str = "right";
    }
    direction dir = (direction)std::atoi(argv[2]);
    std::string dir_str = ((const char*[]) {"left", "right", "up", "down", "front", "back"})[(int) dir];
    log_msg("Moving " + side_str + " arm towards the " + dir_str);
    armTraj.nudgeArmLocal(side, dir);
  }
  else if(argc == 4){
      std::string side_str;
      armSide side;
      if(std::atoi(argv[1]) == 0){
        side = LEFT;
        side_str = "left";
      } else {
        side = RIGHT;
        side_str = "right";
      }
      direction dir = (direction)std::atoi(argv[2]);
      std::string dir_str = ((const char*[]) {"left", "right", "up", "down", "front", "back"})[(int) dir];
      float step_size = std::atof(argv[3]);

      log_msg("Moving " + side_str + " arm towards the " + dir_str + " by " + std::to_string(step_size));
      armTraj.nudgeArmLocal(side, dir, step_size);
    }
    else{
      log_msg("Unexpected arguments, expected side [0: left, 1: right], direction [0: left, 1: right, 2: up, "
                      "3: down, 4: front, 5: back], and optional step size");
      return 1;
  }
  ros::spinOnce();
  ros::Duration(2).sleep();

  log_msg("Motion finished");
  return 0;
}
