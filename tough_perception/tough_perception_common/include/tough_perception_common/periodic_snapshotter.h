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

#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisensePointCloud.h>
#include <tough_perception_common/PointCloudHelper.h>
#include <tough_common/tough_common_names.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <tough_common/robot_state.h>
#include "tough_common/robot_description.h"

namespace laser_assembler
{
enum class PCL_STATE_CONTROL
{
  RESET = 0,
  PAUSE,
  RESUME
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
  void timerCallback(const ros::TimerEvent& e);

  /**
   * @brief mergeClouds merges the pointcloud published on assembled_cloud2 topic with the previous messages
   * that were published on the same topic
   * @param msg
   */
  void mergeClouds(const sensor_msgs::PointCloud2::Ptr msg);

  void pairAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                 Eigen::Matrix4f& final_transform);

  void resetPointcloud(bool resetPointcloud);
  void resetPointcloudCB(const std_msgs::Empty& msg);

  void pausePointcloud(bool pausePointcloud);
  void pausePointcloudCB(const std_msgs::Bool& msg);

  void setBoxFilterCB(const std_msgs::Int8& msg);
  void clipPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

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
  bool first_time_;
  bool downsample_;
  bool resetPointcloud_;
  PCL_STATE_CONTROL state_request;
  bool enable_box_filter_;
  RobotStateInformer* robot_state_;
  RobotDescription* rd_;

  float filter_min_x;
  float filter_max_x;

  float filter_min_y;
  float filter_max_y;

  float filter_min_z;
  float filter_max_z;
};

void convertROStoPCL(const sensor_msgs::PointCloud2::Ptr ros_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*ros_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_msg);
}

void convertPCLtoROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg, sensor_msgs::PointCloud2::Ptr& ros_msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*pcl_msg, pcl_pc2);
  pcl_conversions::moveFromPCL(pcl_pc2, *ros_msg);
}
}

#endif  // PERIODIC_SNAPSHOTTER_H
