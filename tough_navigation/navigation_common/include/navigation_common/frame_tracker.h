#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>
#include <mutex>

/**
 * @brief
 * FRAME_IN_MOTION:
 * If the two tracked frames are tracked and are detected to be in motion, the state is set to this state
 *
 * FRAME_STATIONARY:
 * If the two tracked frames are tracked and are detected not to be in motion, the state is set to this state
 *
 * NOT_TRACKED:
 * If the two frames are not tracked, this state is set
 *
 */
enum class frame_track_status
{
  NOT_TRACKED,
  FRAME_IN_MOTION,
  FRAME_STATIONARY
};

/**
 * @brief This class tracks two frames (frame wrt base_frame) and returns their movement status
 *
 */
class FrameTracker
{
private:
  tf::TransformListener listener_;
  ros::NodeHandle nh_;
  std::string frame_, base_frame_;
  frame_track_status motion_status_;
  std::thread track_thread_;

  void trackFrame(void);
  bool isTranformChanging(tf::StampedTransform transform_curr, tf::StampedTransform transform_prev);
  // threshodl for determining the motion
  const float POSITION_THRESHOLD = 0.01f;          // 0.1cm
  const float ORIENTATION_THRESHOLD = 0.0174533f;  // 1deg

public:

  /**
   * @brief This class tracks two frames (frame wrt base_frame) and returns their movement status
   * 
   * @param nh 
   * @param frame 
   * @param base_frame 
   */
  FrameTracker(ros::NodeHandle nh, std::string frame, std::string base_frame);
  ~FrameTracker();

  /**
   * @brief keeps track of the two frames, and set the frame_track_status according to their movements.
   * This class is usefull for cases like knowing if the given trajectory was executed, or if the robot
   * is done executing the trajectory
   *
   * @return frame_track_status
   */
  frame_track_status isInMotion(void);
};
