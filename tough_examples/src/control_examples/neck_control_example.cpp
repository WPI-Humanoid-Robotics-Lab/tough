#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_neck_navigation");
  ros::NodeHandle nh;
  RobotDescription* rd = RobotDescription::getRobotDescription(nh);
  int n = rd->getNumberOfNeckJoints();
  if (argc != n + 1)
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::stringstream input_trace_filename;
    input_trace_filename << "Usage : rosrun tough_examples " << filename.substr(index + 1);
    for (size_t count = 0; count < n; count++)
    {
      input_trace_filename << " <joint" << count + 1 << "_in_radians> ";
    }
    ROS_INFO_STREAM(input_trace_filename.str());
  }
  else
  {
    std::vector<float> neck_joint_values;
    for (size_t count = 0; count < n; count++)
    {
      neck_joint_values.push_back(std::atof(argv[count + 1]));
    }

    ROS_INFO("Moving the neck");
    std::vector<std::vector<float>> traj_points = { neck_joint_values };
    HeadControlInterface headTraj(nh);
    headTraj.moveNeckJoints(traj_points, 2.0f);

    ros::spinOnce();
    ros::Duration(2).sleep();

    return 0;
  }
}