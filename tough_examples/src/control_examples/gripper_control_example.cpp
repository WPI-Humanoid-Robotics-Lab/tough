#include <tough_controller_interface/gripper_control_interface.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

void demo_gripper(GripperControlInterface& gripcont)
{
  std::vector<GripperControlInterface::GRIPPER_MODES> gripperModes{
    GripperControlInterface::BASIC, GripperControlInterface::PINCH, GripperControlInterface::WIDE,
    GripperControlInterface::SCISSOR, GripperControlInterface::HOOK
  };

  std::vector<RobotSide> gripperSide{ RobotSide::LEFT, RobotSide::RIGHT };
  for (auto side : gripperSide)
  {
    ROS_INFO("[SIDE] %s", (side == RobotSide::LEFT) ? "Left" : "Right");

    for (auto mode : gripperModes)
    {
      ROS_INFO_STREAM("[mode] " << gripcont.getModeName(mode));

      gripcont.setMode(side, mode);

      ros::Duration(2).sleep();

      ROS_INFO("\t closing gripper");
      gripcont.closeGripper(side);

      ros::Duration(2).sleep();

      ROS_INFO("\t opening gripper");
      gripcont.openGripper(side);

      ros::Duration(2).sleep();

      ROS_INFO("\t closing Fingers");
      gripcont.closeFingers(side);

      ros::Duration(2).sleep();

      ROS_INFO("\t opening Fingers");
      gripcont.openFingers(side);

      ros::Duration(2).sleep();

      ROS_INFO("\t closing Thumb");
      gripcont.closeThumb(side);

      ros::Duration(2).sleep();

      ROS_INFO("\t opening Thumb");
      gripcont.openThumb(side);

      ros::Duration(2).sleep();
    }
    ros::Duration(2).sleep();
    ROS_INFO("Resetting gripper");
    //    gripcont.resetGripper(RobotSide::LEFT);
    //    ros::Duration(8).sleep();
    gripcont.setMode(side, GripperControlInterface::BASIC);
    gripcont.closeGripper(side);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_gripper_control");
  ros::NodeHandle nh;
  GripperControlInterface gripcont(nh);
  std::cout << argc << std::endl;

  // wait a reasonable amount of time for the subscriber to connect
  ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
  while (ros::Time::now() < wait_until)
  {
    ros::WallDuration(0.1).sleep();
  }

  std::vector<double> leftGrip, rightGrip;
  if (argc == 2)
  {
    if (argv[1][0] == '1')
    {
      ROS_INFO("opening both grippers");
      gripcont.openGripper(LEFT);
      gripcont.openGripper(RIGHT);
    }
    else
    {
      ROS_INFO("closing both grippers");
      gripcont.closeGripper(LEFT);
      gripcont.closeGripper(RIGHT);
    }
  }
  else if (argc == 3)
  {
    RobotSide side = (RobotSide)std::atoi(argv[1]);
    int state = std::atoi(argv[2]);

    std::string side_str = side == RobotSide::LEFT ? "left" : "right";

    ROS_INFO_STREAM("moving " << side_str << " gripper to " << argv[2]);
    gripcont.controlGripper(side, state);
  }
  else
  {
    std::string filename = std::string(argv[0]);
    int index = filename.find_last_of('/');
    std::string input_trace_filename = filename.substr(index + 1);
    ROS_INFO_STREAM("Usage : rosrun tough_examples " << input_trace_filename
                                                    << " 1 \t\t- to open both the grippers \n"
                                                       " 0 \t\t- to close both the grippers  \n"
                                                       "<side 0=left,1=right> <mode> \t\t refer to hand desired config "
                                                       "message for details");

    demo_gripper(gripcont);
  }
  ros::spinOnce();
  ros::Duration(2).sleep();

  return 0;
}
