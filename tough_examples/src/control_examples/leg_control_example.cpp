#include <ros/ros.h>
#include <stdlib.h>
#include <tough_controller_interface/leg_control_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_control_example");
  ros::NodeHandle nh;
  
  if(argc != 9)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples "
                        << input_trace_filename << " <side_left=0,right=1> <x> <y> <z> <qx> <qy> <qz> <qw>");
    return -1;
  }
  else
  {
    LegControlInterface leg_control(nh);
    RobotSide side = std::atof(argv[1]) == 0 ? RobotSide::LEFT : RobotSide::RIGHT;
    double time = 1.0;

    geometry_msgs::Pose leg_pose;
    leg_pose.position.x = std::atof(argv[2]); 
    leg_pose.position.y = std::atof(argv[3]);
    leg_pose.position.z = std::atof(argv[4]);

    leg_pose.orientation.x = std::atof(argv[5]);
    leg_pose.orientation.y = std::atof(argv[6]); 
    leg_pose.orientation.z = std::atof(argv[7]);
    leg_pose.orientation.w = std::atof(argv[8]);

    std::vector<geometry_msgs::Pose> leg_control_vec = { leg_pose };

    ros::Duration(time).sleep();
    leg_control.moveFoot(side, leg_control_vec, time);
    ros::Duration(time).sleep();
  }
  return 0;
}
