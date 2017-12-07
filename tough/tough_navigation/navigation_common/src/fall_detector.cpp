#include <navigation_common/fall_detector.h>

FallDetector::FallDetector(ros::NodeHandle nh, std::string foot_frame, std::string root_frame, std::string world_frame):
  nh_(nh), foot_frame_(foot_frame), root_frame_(root_frame), world_frame_(world_frame)
{
  // robot is not fallen on init
  isrobot_fallen_ = false;

  // set the listener
  try
  {
    tf::StampedTransform foot_transform, root_transform;
    foot_listener_.waitForTransform(world_frame_, foot_frame_,  ros::Time(0), ros::Duration(5.0));
    foot_listener_.lookupTransform(world_frame_,foot_frame_,  ros::Time(0), foot_transform);

    root_listener_.waitForTransform(world_frame_, root_frame_,  ros::Time(0), ros::Duration(5.0));
    root_listener_.lookupTransform(world_frame_, root_frame_,  ros::Time(0), root_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  falltrack_thread_ = std::thread(&FallDetector::trackRobotFall, this);
  falltrack_thread_.detach();
}


FallDetector::~FallDetector(){
}

bool FallDetector::isRobotFallen()
{
  return isrobot_fallen_;
}

void  FallDetector::trackRobotFall(void)
{
  ros::Rate rate(10.0);
  tf::StampedTransform foot_transform, root_transform;

  while (ros::ok())
  {
    foot_listener_.lookupTransform(world_frame_, foot_frame_, ros::Time(0), foot_transform);
    root_listener_.lookupTransform(world_frame_, root_frame_, ros::Time(0), root_transform);

    //    std::cout << fabs(fabs(root_transform.getOrigin().getZ()) - fabs(foot_transform.getOrigin().getZ())) << std::endl;

    if (fabs(fabs(root_transform.getOrigin().getZ()) - fabs(foot_transform.getOrigin().getZ())) < ZTHRESHOLD)
    {
      isrobot_fallen_ = true;
    }
    else
    {
      isrobot_fallen_ = false;
    }

    rate.sleep();
  }
}
