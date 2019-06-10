#include <iostream>
#include <tough_footstep/robot_walker.h>
#include <tough_control_common/tough_control_common.h>
#include <ihmc_msgs/FootstepDataListRosMessage.h>
#include "geometry_msgs/Pose2D.h"
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>

using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "walk_test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  RobotWalker walk(nh, 1.0, 1.0, 0);
  ToughControlCommon tough_contol_common(nh);

  char input;
  int robot_side;
  float stepHeight;
  float stepLength = 0.0;
  float curl;
  float nudgeDistance;
  float offset;

  //    geometry_msgs::Pose2D goal;
  //    goal.x=0.0;
  //    goal.y=0.0;
  //    goal.theta=-1.57;
  //    walk.walkToGoal(goal);

  //    std::vector<float> x_offset,y_offset;
  //    x_offset={0.2,0.2};
  //    y_offset={0.0,0.0};
  //    walk.walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);

  while (ros::ok())
  {
    cout << "************ ************ ************ \n";
    cout << "enter choice \n";
    cout << "q - exit code \n";
    cout << "s - stop trajectories \n";
    cout << "l - load \n";
    cout << "u - up \n";
    cout << "c - curl \n";
    cout << "f - forward \n";
    cout << "m - move foot to given pose in world frame \n";
    cout << "p - place leg \n";
    cout << "a - align feet\n";

    cin >> input;
    if (input == 'q')
    {
      cout << "Exiting Code \n";
      exit(0);
    }
    else if (input == 'u')
    {
      cout << "enter side \n";
      cin >> robot_side;
      cout << "enter step height \n";
      cin >> stepHeight;
      //            cout<<"enter step length \n";
      //            cin>>stepLength;
      walk.raiseLeg((RobotSide)robot_side, stepHeight, stepLength);
    }
    else if (input == 'l')
    {
      cout << "enter side \n";
      cin >> robot_side;
      walk.loadEEF((RobotSide)robot_side, EE_LOADING::LOAD);
    }
    else if (input == 'c')
    {
      cout << "enter side \n";
      cin >> robot_side;
      cout << "enter curl radius \n";
      cin >> curl;
      walk.curlLeg((RobotSide)robot_side, curl);
    }
    else if (input == 'p')
    {
      cout << "enter side \n";
      cin >> robot_side;
      cout << "enter offset \n";
      cin >> offset;
      walk.placeLeg((RobotSide)robot_side, offset);
    }
    else if (input == 'f')
    {
      cout << "enter side \n";
      cin >> robot_side;
      cout << "enter forward distance \n";
      cin >> nudgeDistance;
      walk.nudgeFoot((RobotSide)robot_side, nudgeDistance);
    }
    else if (input == 'm')
    {
      std::cout << "Enter <side> <x> <y> <z> <ax> <ay> <az> <aw>:";
      geometry_msgs::Pose pt;
      std::cin >> robot_side >> pt.position.x >> pt.position.y >> pt.position.z >> pt.orientation.x >>
          pt.orientation.y >> pt.orientation.z >> pt.orientation.w;
      double t;
      std::cout << "Enter time to reach that pose : ";
      std::cin >> t;
      walk.moveFoot((RobotSide)robot_side, pt, t);
    }
    else if (input == 's')
    {
      tough_contol_common.stopAllTrajectories();
      cout << "Stopped All Trajectories \n";
    }
    else if (input == 'a')
    {
      walk.alignFeet(RobotSide::RIGHT);
    }
    else
      cout << "invalid input \n";
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
