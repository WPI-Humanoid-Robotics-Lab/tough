#include <val_descartes_planner/val_descartes_palnner.h>

typedef std::vector<descartes_core::TrajectoryPtPtr>::const_iterator TrajectoryIter;

void generateDPoints(std::vector<geometry_msgs::Pose>& pose, std::vector<descartes_core::TrajectoryPtPtr>& wayPoints);

void dplanner::toROSJointTrajectory(const std::vector<descartes_core::TrajectoryPtPtr>& trajectory,
                                    const descartes_core::RobotModel& model,
                                    const std::vector<std::string>& joint_names, double time_delay,
                                    trajectory_msgs::JointTrajectory& traj)
{
  // Fill out information about our trajectory
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "world";
  traj.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    traj.points.push_back(pt);
  }
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  ROS_INFO("a1");
  return TrajectoryPtPtr(new AxialSymmetricPt(pose, M_PI / 2.0 - 0.0001, AxialSymmetricPt::Z_AXIS));
}

void generateDPoints(std::vector<geometry_msgs::Pose>& pose, std::vector<descartes_core::TrajectoryPtPtr>& wayPoints)
{
  for (int i = 0; i < pose.size() - 1; ++i)
  {
    ROS_INFO("1");
    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose[i], eigen_pose);

    // eigen_pose = Eigen::Translation3d(0.0, 0.0, 1.0 + 0.05 * i); //points[i].position.x, points[i].position.y,
    // points[i].position.z);
    //        std::cout << "pose trans " << eigen_pose.translation();
    ROS_INFO("2");
    //        Eigen::Quaternion<double> q(pose[i].orientation.w, pose[i].orientation.x, pose[i].orientation.y,
    //        pose[i].orientation.z);
    //        eigen_pose.linear() = q.toRotationMatrix();

    ROS_INFO("3");
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(eigen_pose);
    ROS_INFO("4");
    wayPoints.push_back(pt);
    ROS_INFO("5");
  }
}

void dplanner::plantrajectory(ros::NodeHandle nh, std::vector<geometry_msgs::Pose>& pose,
                              trajectory_msgs::JointTrajectory& traj)
{
  // Required for communication with moveit components
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<descartes_core::TrajectoryPtPtr> waypoints;
  generateDPoints(pose, waypoints);

  ROS_INFO("generated waypoints");

  // Create a robot model and initialize it
  descartes_core::RobotModelPtr model(new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "rightArm";

  // Name of frame in which you are expressing poses.
  const std::string world_frame = "/world";

  // tool center point frame (name of link associated with tool)
  const std::string tcp_frame = "rightPalm";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
  }

  std::cout << "DOF's " << model->getDOF() << std::endl;
  // Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  ROS_INFO("initialised planner");

  // Feed the trajectory to the planner

  descartes_core::PlannerConfig config;
  planner.getConfig(config);
  for (std::map<std::string, std::string>::iterator it = config.begin(); it != config.end(); ++it)
    std::cout << it->first << " => " << it->second << '\n';

  if (!planner.planPath(waypoints))
  {
    ROS_ERROR("Could not solve for a valid path");
  }

  ROS_INFO("planning done");
  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
  }

  planner.getPlanningGraph();
  // 5. Translate the result into a type that ROS understands
  // Get Joint Names
  std::string robot_name;
  nh.getParam("ihmc_ros/robot_name", robot_name);

  std::vector<std::string> names;
  nh.getParam("/ihmc_ros/" + robot_name + "/right_arm_joint_names", names);
  // Generate a ROS joint trajectory with the result path, robot model,
  // joint names, and a certain time delta between each trajectory point
  toROSJointTrajectory(result, *model, names, 1.0, traj);

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
}
