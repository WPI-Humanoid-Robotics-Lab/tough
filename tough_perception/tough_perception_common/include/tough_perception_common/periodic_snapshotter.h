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

#include <iostream>
#include <algorithm>

namespace laser_assembler
{
enum class PCL_STATE_CONTROL
{
  RESET = 0,
  PAUSE,
  RESUME
};

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointTI;
typedef sensor_msgs::PointCloud2 PointCloudSensorMsg;

typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointTI> PointCloud_I;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/****************************************************
 * custom class required for pointcloud registeration
 ****************************************************/
// Define a new point representation for < x, y, z, curvature >
class customPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
  customPointRepresentation()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const PointNormalT &p, float *out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
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

  void pairAlign(const PointCloud::Ptr cloud_src,
                 const PointCloud::Ptr cloud_tgt,
                 PointCloud::Ptr output);

  void pairAlign_I(const PointCloud_I::Ptr cloud_src,
                   const PointCloud_I::Ptr cloud_tgt,
                   PointCloud_I::Ptr output);

  void resetPointcloud(bool resetPointcloud);
  void resetPointcloudCB(const std_msgs::Empty &msg);

  void pausePointcloud(bool pausePointcloud);
  void pausePointcloudCB(const std_msgs::Bool &msg);

  void setBoxFilterCB(const std_msgs::Int8 &msg);
  void clipPointCloud(const PointCloud::Ptr input_cloud);
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
  PointCloud::Ptr assembled_pc;
  PointCloud_I::Ptr assembled_pc_I;

  bool first_time_;
  bool downsample_;
  bool resetPointcloud_;
  bool enable_box_filter_;

  PCL_STATE_CONTROL state_request;
  RobotStateInformer *robot_state_;
  RobotDescription *rd_;

  float filter_min_x;
  float filter_max_x;

  float filter_min_y;
  float filter_max_y;

  float filter_min_z;
  float filter_max_z;

  int snapshotCount_;
  const int MAX_SNAPSHOTS = 10;
};

template <class T, class U>
// void convertROStoPCL(const sensor_msgs::PointCloud2::Ptr ros_msg, pcl::PointCloud<T>::Ptr &pcl_msg)
void convertROStoPCL(const U ros_msg, T &pcl_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*ros_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_msg);
}

template <class T, class U>
// void convertPCLtoROS(const pcl::PointCloud<T>::Ptr pcl_msg, sensor_msgs::PointCloud2::Ptr &ros_msg)
void convertPCLtoROS(const T pcl_msg, U &ros_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pcl_msg, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, *ros_msg);
}

void addIntensity(const PointCloud::Ptr pc1, PointCloud_I::Ptr pc2)
{
  copyPointCloud(*pc1, *pc2);
  for (size_t i = 0; i < pc2->points.size(); i++)
  {
    pc2->points[i].intensity = 1.0f;
  }
  // std::for_each(pc2->points.begin(), pc2->points.end(), [](PointI &p) { p.intensity = 1.0f; })
}

void decayPoint(PointCloud_I::Ptr pc, float step = 0.1)
{
  for (size_t i = 0; i < pc->size(); i++)
  {
    if (pc->points[i].intensity > 0.0f)
      pc->points[i].intensity -= step;
  }
  // std::for_each(pc->points.begin(), pc->points.end(), [&step](PointI &p) { if (p.intensity > 0.0f) p.intensity -= step; })
}

void min_internsity(PointCloud_I::Ptr pc)
{
  PointTI x = *std::min_element(pc->points.begin(),
                                pc->points.end(),
                                [](PointTI i, PointTI j) { return i.intensity < j.intensity; });
  std::cout << "[MIN INTENSITY] " << x.intensity << std::endl;
}

void filterDeadPointCloud(PointCloud_I::Ptr pc, float dead_threshold = 0.0f)
{
  auto n_end = pc->points.end();
  auto p_end = std::remove_if(pc->points.begin(),
                              pc->points.end(),
                              [&dead_threshold](PointTI i) { return i.intensity <= dead_threshold; });
  pc->points.erase(p_end, n_end);
}

} // namespace laser_assembler

#endif // PERIODIC_SNAPSHOTTER_H
