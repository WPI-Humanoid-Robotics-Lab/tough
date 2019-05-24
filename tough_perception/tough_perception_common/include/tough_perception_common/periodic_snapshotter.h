/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef PERIODIC_SNAPSHOTTER_H
#define PERIODIC_SNAPSHOTTER_H

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_representation.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <tough_common/tough_common_names.h>
#include <tough_perception_common/global.h>
#include <tough_common/robot_state.h>
#include <tough_common/robot_description.h>
#include <tough_perception_common/PerceptionHelper.h>

#include <iostream>
#include <algorithm>

namespace laser_assembler
{
using namespace tough_perception;

/**
 * @brief Enum class which holds the control states of the snapshotter
 * 
 */
enum class PCL_STATE_CONTROL
{
  RESET = 0,
  PAUSE,
  RESUME
};

enum class BOX_FILTER_TYPE
{
  WAIST_UP = 1,
  LARGE_BOX = 2,
};

class PeriodicSnapshotter
{
public:
  /**
   * @brief PeriodicSnapshotter requests a point cloud from the
   * point_cloud_assembler every x seconds, and then publishes the
   * resulting data
   */
  PeriodicSnapshotter();

  /**
   * @brief timerCallback This callback is executed after a set timeout. This timeout is specified
   * in the launch file. example: <param name="/laser_assembler_svc/laser_snapshot_timeout" type="double" value="6.0"/>
   * @param e
   */
  void timerCallback(const ros::TimerEvent &e);

  /**
   * @brief mergeClouds merges the pointcloud published on assembled_cloud2 topic with the previous messages
   * that were published on the same topic
   * @param msg
   */
  void mergeClouds(const sensor_msgs::PointCloud2::Ptr msg);

  /**
   * @brief Aligns the source and target point cloud using Iterative Closest Point algorithm 
   * 
   * @param cloud_src 
   * @param cloud_tgt 
   * @param output 
   */
  void pairAlign_I(const PointCloud_I::Ptr cloud_src,
                   const PointCloud_I::Ptr cloud_tgt,
                   PointCloud_I::Ptr output);

  void addIntensity(const PointCloud::Ptr pc1, PointCloud_I::Ptr pc2);
  void decayPoint(PointCloud_I::Ptr pc, float step = 0.1);
  void filterDeadPointCloud(PointCloud_I::Ptr pc, float dead_threshold = 0.0f);

  void resetPointcloud(bool resetPointcloud);
  void resetPointcloudCB(const std_msgs::Empty &msg);

  void pausePointcloud(bool pausePointcloud);
  void pausePointcloudCB(const std_msgs::Bool &msg);

  void setBoxFilterCB(const std_msgs::Int8 &msg);
  void clipPointCloud(const PointCloud_I::Ptr input_cloud);

private:
  ros::NodeHandle n_;
  ros::Publisher snapshot_pub_;
  ros::Publisher registered_pointcloud_pub_;
  ros::Publisher pointcloud_for_octomap_pub_;
  ros::Subscriber snapshot_sub_;
  ros::Subscriber resetPointcloudSub_;
  ros::Subscriber pausePointcloudSub_;
  ros::Subscriber boxFilterSub_;
  ros::ServiceClient client_;
  ros::Timer timer_;

  sensor_msgs::PointCloud2::Ptr prev_msg_;
  PointCloud_I::Ptr assembled_pc_I;

  PassThroughFilter<PointTI> pass_through_filt;
  VoxelGridFilter<PointTI> voxel_grid_filt_0_05;

  bool first_time_;
  bool downsample_;
  bool resetPointcloud_;
  bool enable_box_filter_;

  PCL_STATE_CONTROL state_request;
  RobotStateInformer *robot_state_;
  RobotDescription *rd_;

  float filter_min_x, filter_max_x;
  float filter_min_y, filter_max_y;
  float filter_min_z, filter_max_z;

  int snapshotCount_;
  const int MAX_SNAPSHOTS = 10;
};

} // namespace laser_assembler

#endif // PERIODIC_SNAPSHOTTER_H
