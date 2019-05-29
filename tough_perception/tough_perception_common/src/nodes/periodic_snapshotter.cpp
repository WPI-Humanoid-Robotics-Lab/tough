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
// #define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <cstdio>
#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_density.h>

#include <pcl/visualization/pcl_visualizer.h>

// Services
#include <laser_assembler/AssembleScans2.h>

// Messages
#include <sensor_msgs/PointCloud2.h>

#include <tough_perception_common/periodic_snapshotter.h>
#include <tough_perception_common/PerceptionHelper.h>

/***
 * This used to be a simple test app that requests a point cloud from the
 * point_cloud_assembler every x seconds, and then publishes the
 * resulting data
 */

using namespace laser_assembler;
using namespace tough_perception;

PeriodicSnapshotter::PeriodicSnapshotter() : assembled_pc_I(new PointCloud_I),
                                             prev_msg_(new sensor_msgs::PointCloud2)
{
  robot_state_ = RobotStateInformer::getRobotStateInformer(n_);
  rd_ = RobotDescription::getRobotDescription(n_);

  // Setting all publishers
  snapshot_pub_ = n_.advertise<sensor_msgs::PointCloud2>("snapshot_cloud2", 1, true);
  registered_pointcloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_cloud2", 1, true);
  pointcloud_for_octomap_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_octomap_cloud2", 10, true);

  // Setting all subscribers
  resetPointcloudSub_ = n_.subscribe("reset_pointcloud", 10, &PeriodicSnapshotter::resetPointcloudCB, this);
  pausePointcloudSub_ = n_.subscribe("pause_pointcloud", 10, &PeriodicSnapshotter::pausePointcloudCB, this);
  boxFilterSub_ = n_.subscribe("clearbox_pointcloud", 10, &PeriodicSnapshotter::setBoxFilterCB, this);

  // Create the service client for calling the assembler
  client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");
  snapshot_sub_ = n_.subscribe("snapshot_cloud2", 10, &PeriodicSnapshotter::mergeClouds, this);

  // Start the timer that will trigger the processing loop (timerCallback)
  float timeout;
  n_.param<float>("laser_assembler_svc/laser_snapshot_timeout", timeout, 5.0);
  ROS_INFO("PeriodicSnapshotter::PeriodicSnapshotter : Snapshot timeout : %.2f seconds", timeout);
  timer_ = n_.createTimer(ros::Duration(timeout, 0), &PeriodicSnapshotter::timerCallback, this);

  // Need to track if we've called the timerCallback at least once
  first_time_ = true;
  downsample_ = true;
  enable_box_filter_ = false;
  resetPointcloud_ = true;

  n_.param<float>("filter_min_x", filter_min_x, -10.0);
  n_.param<float>("filter_max_x", filter_max_x, 10.0);

  n_.param<float>("filter_min_y", filter_min_y, -10.0);
  n_.param<float>("filter_max_y", filter_max_y, 10.0);

  n_.param<float>("filter_min_z", filter_min_z, -10.0);
  n_.param<float>("filter_max_z", filter_max_z, 10.0);
  snapshotCount_ = 0;
}

void PeriodicSnapshotter::timerCallback(const ros::TimerEvent &e)
{
  // We don't want to build a cloud the first callback, since we
  //   don't have a start and end time yet
  if (first_time_)
  {
    ROS_INFO("PeriodicSnapshotter::timerCallback : Ignoring current snapshot");
    first_time_ = false;
    return;
  }

  // Populate our service request based on our timer callback times
  AssembleScans2 srv;
  srv.request.begin = e.last_real;
  srv.request.end = e.current_real;

  // Make the service call
  if (client_.call(srv))
  {
    ROS_INFO("PeriodicSnapshotter::timerCallback : Published Cloud with %u points",
             (uint32_t)(srv.response.cloud.data.size()));
    snapshot_pub_.publish(srv.response.cloud);
    ++snapshotCount_;
    if (snapshotCount_ > MAX_SNAPSHOTS)
    {
      pausePointcloud(true);
    }
  }
  else
  {
    ROS_WARN("Error making service call\n");
  }
}

void PeriodicSnapshotter::resetPointcloud(bool resetPointcloud)
{
  // reset the assembler
  state_request = PCL_STATE_CONTROL::RESET;
  snapshotCount_ = 0;
}

void PeriodicSnapshotter::resetPointcloudCB(const std_msgs::Empty &msg)
{
  state_request = PCL_STATE_CONTROL::RESET;
  snapshotCount_ = 0;
}

void PeriodicSnapshotter::pausePointcloud(bool pausePointcloud)
{
  state_request = pausePointcloud ? PCL_STATE_CONTROL::PAUSE : PCL_STATE_CONTROL::RESUME;
  if (!pausePointcloud)
  {
    snapshotCount_ = 0;
  }
}

void PeriodicSnapshotter::pausePointcloudCB(const std_msgs::Bool &msg)
{
  // reset will make sure that older scans are discarded
  state_request = msg.data ? PCL_STATE_CONTROL::PAUSE : PCL_STATE_CONTROL::RESUME;
  if (!msg.data)
  {
    snapshotCount_ = 0;
  }
}

void PeriodicSnapshotter::setBoxFilterCB(const std_msgs::Int8 &msg)
{
  enable_box_filter_ = true;

  pcl::PointCloud<PointT>::Ptr pcl_prev_msg(new pcl::PointCloud<PointT>);
  // convertROStoPCL<PointCloud::Ptr, PointCloudSensorMsg::Ptr>(prev_msg_, pcl_prev_msg);
  convertROStoPCL<PointCloud::Ptr>(prev_msg_, pcl_prev_msg);

  geometry_msgs::Pose pelvisPose;
  pcl::PointCloud<PointT>::Ptr tgt(new pcl::PointCloud<PointT>);
  robot_state_->getCurrentPose(rd_->getPelvisFrame(), pelvisPose);

  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;

  // this indicates that if the msg contains element 1 it, would clear point cloud from waist up
  if (msg.data == (int8_t)BOX_FILTER_TYPE::WAIST_UP) // waist up condition
  {
    // <x,y,z>
    minPoint << -1, -1, 0.0;
    maxPoint << 2, 1, 1;
  }
  else if (msg.data == (int8_t)BOX_FILTER_TYPE::LARGE_BOX) // large box condition
  {
    minPoint << -1.0, -1.5, -0.5;
    maxPoint << 4, 1.5, 1.5;
  }
  else // full box condition
  {
    minPoint << -1, -1, -0.5;
    maxPoint << 2, 1, 1;
  }

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = pelvisPose.position.x;
  boxTranslatation[1] = pelvisPose.position.y;
  boxTranslatation[2] = pelvisPose.position.z;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0;                                  // rotation around x-axis
  boxRotation[1] = 0;                                  // rotation around y-axis
  boxRotation[2] = tf::getYaw(pelvisPose.orientation); // in radians rotation around z-axis. this rotates your cube
                                                       // 45deg around z-axis.

  pcl::CropBox<PointT> box_filter;
  box_filter.setInputCloud(pcl_prev_msg);
  box_filter.setMin(minPoint);
  box_filter.setMax(maxPoint);
  box_filter.setTranslation(boxTranslatation);
  box_filter.setRotation(boxRotation);
  box_filter.setNegative(true);
  box_filter.filter(*tgt);

  sensor_msgs::PointCloud2::Ptr merged_cloud(new sensor_msgs::PointCloud2());
  convertPCLtoROS<PointCloud::Ptr>(tgt, merged_cloud);
  prev_msg_ = merged_cloud;

  registered_pointcloud_pub_.publish(merged_cloud);
  enable_box_filter_ = false;
}

void PeriodicSnapshotter::mergeClouds(const PointCloudSensorMsg::Ptr msg)
{
  if (enable_box_filter_)
    return;

  if (state_request == PCL_STATE_CONTROL::PAUSE)
  {
    ROS_INFO("PeriodicSnapshotter::mergeClouds : Laser assembling paused");
    registered_pointcloud_pub_.publish(prev_msg_);
    return;
  }

  PointCloudSensorMsg::Ptr merged_cloud(new PointCloudSensorMsg());
  PointCloud::Ptr pcl_msg(new PointCloud);
  PointCloud_I::Ptr pclI_msg(new PointCloud_I);

  // convertROStoPCL<PointCloud::Ptr, PointCloudSensorMsg::Ptr>(msg, pcl_msg);
  convertROStoPCL<PointCloud::Ptr>(msg, pcl_msg);
  addIntensity(pcl_msg, pclI_msg); // add initial value

  if (state_request == PCL_STATE_CONTROL::RESET || prev_msg_->data.empty())
  {
    ROS_INFO("PeriodicSnapshotter::mergeClouds : Resetting Pointcloud");
    merged_cloud = msg;
    assembled_pc_I = pclI_msg;
    pointcloud_for_octomap_pub_.publish(prev_msg_->data.empty() ? msg : prev_msg_);
    state_request = PCL_STATE_CONTROL::RESUME;
  }
  else
  {
    // merge the current msg with previous messages published till now
    // http://www.pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration

    PointCloud_I::Ptr result_I(new PointCloud_I);
    align_point_clouds(pclI_msg, assembled_pc_I, result_I);

    clipPointCloud(result_I); // clip the point cloud in x y and z direction

    ROS_INFO("PeriodicSnapshotter::mergeClouds : PC size : %d", result_I->size());

    decayPoint(result_I);

    convertPCLtoROS<PointCloud_I::Ptr>(result_I, merged_cloud);
    assembled_pc_I = result_I;

    ROS_INFO("PeriodicSnapshotter::mergeClouds : assembled PC size %d", assembled_pc_I->size());
    filterDeadPointCloud(assembled_pc_I, 0.5f);
    ROS_INFO("PeriodicSnapshotter::mergeClouds : assembled  PC size filtered %d", assembled_pc_I->size());
  }

  // publish the merged message
  prev_msg_ = merged_cloud;
  registered_pointcloud_pub_.publish(merged_cloud);
}

void PeriodicSnapshotter::clipPointCloud(const PointCloud_I::Ptr input_cloud)
{
  pass_through_filt(input_cloud, "z", filter_min_z, filter_max_z);
  pass_through_filt(input_cloud, "y", filter_min_y, filter_max_y);
  pass_through_filt(input_cloud, "x", filter_min_x, filter_max_x);
}

void PeriodicSnapshotter::addIntensity(const PointCloud::Ptr pc1, PointCloud_I::Ptr pc2)
{
  copyPointCloud(*pc1, *pc2);
  std::for_each(pc2->points.begin(),
                pc2->points.end(),
                [](PointTI &p) { p.intensity = 1.0f; });
}

void PeriodicSnapshotter::decayPoint(PointCloud_I::Ptr pc, float step)
{
  std::for_each(pc->points.begin(),
                pc->points.end(),
                [&step](PointTI &p) {
                  if (p.intensity > 0.0f)
                    p.intensity -= step;
                });
}

void PeriodicSnapshotter::filterDeadPointCloud(PointCloud_I::Ptr pc, float dead_threshold)
{
  auto n_end = pc->points.end();
  auto p_end = std::remove_if(pc->points.begin(),
                              pc->points.end(),
                              [&dead_threshold](PointTI i) { return i.intensity <= dead_threshold; });
  pc->points.erase(p_end, n_end);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;

  float timeout;
  n.param<float>("laser_assembler_svc/laser_snapshot_timeout", timeout, 5.0);

  ros::Rate looprate((double)timeout);
  while (ros::ok())
  {
    ros::spinOnce();
    looprate.sleep();
  }
  return 0;
}
