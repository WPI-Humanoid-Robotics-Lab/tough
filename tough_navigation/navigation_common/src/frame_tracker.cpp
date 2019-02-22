#include <navigation_common/frame_tracker.h>

FrameTracker::FrameTracker(ros::NodeHandle nh, std::string frame, std::string base_frame)
  : nh_(nh), frame_(frame), base_frame_(base_frame)
{
  motion_status_ = frame_track_status::NOT_TRACKED;

  // set the listener
  try
  {
    tf::StampedTransform transform;
    listener_.waitForTransform(frame_, base_frame_, ros::Time(0), ros::Duration(5.0));
    listener_.lookupTransform(frame_, base_frame_, ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  track_thread_ = std::thread(&FrameTracker::trackFrame, this);
  track_thread_.detach();
}

FrameTracker::~FrameTracker()
{
  ROS_INFO("destructor");
}

frame_track_status FrameTracker::isInMotion()
{
  return motion_status_;
}

void FrameTracker::trackFrame(void)
{
  ros::Rate rate(10.0);
  tf::StampedTransform transform, transform_prev;

  while (ros::ok())
  {
    listener_.lookupTransform(frame_, base_frame_, ros::Time(0), transform);
    if (isTranformChanging(transform, transform_prev))
    {
      motion_status_ = frame_track_status::FRAME_IN_MOTION;
    }
    else
    {
      motion_status_ = frame_track_status::FRAME_STATIONARY;
    }

    // update the previous transform
    transform_prev = transform;

    rate.sleep();
  }
}

bool FrameTracker::isTranformChanging(tf::StampedTransform transform_curr, tf::StampedTransform transform_prev)
{
  bool ret = false;
  tf::StampedTransform transform_diff;
  // get the diff between the transforms
  transform_diff.setOrigin(transform_curr.getOrigin() - transform_prev.getOrigin());
  transform_diff.setRotation(transform_curr.getRotation() - transform_prev.getRotation());

  // if the transform changes
  if (fabs((transform_curr.getOrigin() - transform_prev.getOrigin()).getX()) > POSITION_THRESHOLD ||
      fabs((transform_curr.getOrigin() - transform_prev.getOrigin()).getY()) > POSITION_THRESHOLD ||
      fabs((transform_curr.getOrigin() - transform_prev.getOrigin()).getZ()) > POSITION_THRESHOLD ||
      fabs((transform_curr.getRotation() - transform_prev.getRotation()).getX()) > ORIENTATION_THRESHOLD ||
      fabs((transform_curr.getRotation() - transform_prev.getRotation()).getY()) > ORIENTATION_THRESHOLD ||
      fabs((transform_curr.getRotation() - transform_prev.getRotation()).getZ()) > ORIENTATION_THRESHOLD ||
      fabs((transform_curr.getRotation() - transform_prev.getRotation()).getW()) > ORIENTATION_THRESHOLD)
  {
    ret = true;
  }

  return ret;
}
