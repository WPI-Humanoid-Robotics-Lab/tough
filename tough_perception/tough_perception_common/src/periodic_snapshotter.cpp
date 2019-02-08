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

#include <cstdio>
#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

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
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"

#include "tough_perception_common/periodic_snapshotter.h"

/***
 * This used to be a simple test app that requests a point cloud from the
 * point_cloud_assembler every x seconds, and then publishes the
 * resulting data
 */

using namespace laser_assembler;

/****************************************************
 * namespaces and typedefs required for registeration
 ****************************************************/
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
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
  virtual void copyToFloatArray(const PointNormalT& p, float* out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

PeriodicSnapshotter::PeriodicSnapshotter()
{
  robot_state_ = RobotStateInformer::getRobotStateInformer(n_);
  rd_ = RobotDescription::getRobotDescription(n_);
  snapshot_pub_ = n_.advertise<sensor_msgs::PointCloud2>("snapshot_cloud2", 1, true);
  registered_pointcloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_cloud2", 1, true);
  pointcloud_for_octomap_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_octomap_cloud2", 10, true);
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
  sensor_msgs::PointCloud2::Ptr temp(new sensor_msgs::PointCloud2);
  prev_msg_ = temp;
  resetPointcloud_ = true;

  n_.param<float>("filter_min_x", filter_min_x, -10.0);
  n_.param<float>("filter_max_x", filter_max_x, 10.0);

  n_.param<float>("filter_min_y", filter_min_y, -10.0);
  n_.param<float>("filter_max_y", filter_max_y, 10.0);

  n_.param<float>("filter_min_z", filter_min_z, -10.0);
  n_.param<float>("filter_max_z", filter_max_z, 10.0);
}

void PeriodicSnapshotter::timerCallback(const ros::TimerEvent& e)
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
  }
  else
  {
    ROS_WARN("Error making service call\n");
  }
}

void PeriodicSnapshotter::pairAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f& final_transform)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize(0.05, 0.05, 0.05);
  grid.setInputCloud(cloud_src);
  grid.filter(*src);

  grid.setInputCloud(cloud_tgt);
  grid.filter(*tgt);

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  customPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const customPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  //
  // Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(2);
  for (int i = 0; i < 30; ++i)
  {
    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();
  }

  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);

  // add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
}

void PeriodicSnapshotter::resetPointcloud(bool resetPointcloud)
{
  // reset the assembler
  state_request = PCL_STATE_CONTROL::RESET;
}

void PeriodicSnapshotter::resetPointcloudCB(const std_msgs::Empty& msg)
{
  state_request = PCL_STATE_CONTROL::RESET;
}

void PeriodicSnapshotter::pausePointcloud(bool pausePointcloud)
{
  state_request = pausePointcloud ? PCL_STATE_CONTROL::PAUSE : PCL_STATE_CONTROL::RESUME;
}

void PeriodicSnapshotter::pausePointcloudCB(const std_msgs::Bool& msg)
{
  // reset will make sure that older scans are discarded
  state_request = msg.data ? PCL_STATE_CONTROL::PAUSE : PCL_STATE_CONTROL::RESUME;
}

void PeriodicSnapshotter::setBoxFilterCB(const std_msgs::Int8& msg)
{
  enable_box_filter_ = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_prev_msg(new pcl::PointCloud<pcl::PointXYZ>);
  convertROStoPCL(prev_msg_, pcl_prev_msg);
  geometry_msgs::Pose pelvisPose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
  robot_state_->getCurrentPose(rd_->getPelvisFrame(), pelvisPose);
  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;

  // this indicates that if the msg contains element 1 it, would clear point cloud from waist up
  if (msg.data == 1)
  {
    // waist up condition
    minPoint[0] = -1;   // x
    minPoint[1] = -1;   // y
    minPoint[2] = 0.0;  // z

    maxPoint[0] = 2;
    maxPoint[1] = 1;
    maxPoint[2] = 1;
  }
  else if (msg.data == 2)
  {
    // large box condition
    minPoint[0] = -1.0;
    minPoint[1] = -1.5;
    minPoint[2] = -0.5;

    maxPoint[0] = 4;
    maxPoint[1] = 1.5;
    maxPoint[2] = 1.5;
  }
  else
  {
    // full box condition
    minPoint[0] = -1;
    minPoint[1] = -1;
    minPoint[2] = -0.5;

    maxPoint[0] = 2;
    maxPoint[1] = 1;
    maxPoint[2] = 1;
  }

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = pelvisPose.position.x;
  boxTranslatation[1] = pelvisPose.position.y;
  boxTranslatation[2] = pelvisPose.position.z;
  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0;  // rotation around x-axis
  boxRotation[1] = 0;  // rotation around y-axis
  boxRotation[2] = tf::getYaw(
      pelvisPose.orientation);  // in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

  pcl::CropBox<pcl::PointXYZ> box_filter;
  std::vector<int> indices;
  indices.clear();
  box_filter.setInputCloud(pcl_prev_msg);
  box_filter.setMin(minPoint);
  box_filter.setMax(maxPoint);
  box_filter.setTranslation(boxTranslatation);
  box_filter.setRotation(boxRotation);
  box_filter.setNegative(true);
  box_filter.filter(*tgt);
  sensor_msgs::PointCloud2::Ptr merged_cloud(new sensor_msgs::PointCloud2());
  convertPCLtoROS(tgt, merged_cloud);
  prev_msg_ = merged_cloud;

  registered_pointcloud_pub_.publish(merged_cloud);
  enable_box_filter_ = false;
}

void PeriodicSnapshotter::mergeClouds(const sensor_msgs::PointCloud2::Ptr msg)
{
  if (enable_box_filter_)
    return;

  if (state_request == PCL_STATE_CONTROL::PAUSE)
  {
    ROS_INFO("PeriodicSnapshotter::mergeClouds : Laser assembling paused");
    registered_pointcloud_pub_.publish(prev_msg_);
    return;
  }

  sensor_msgs::PointCloud2::Ptr merged_cloud(new sensor_msgs::PointCloud2());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
  convertROStoPCL(msg, pcl_msg);

  if (state_request == PCL_STATE_CONTROL::RESET || prev_msg_->data.empty())
  {
    ROS_INFO("PeriodicSnapshotter::mergeClouds : Resetting Pointcloud");
    merged_cloud = msg;
    pointcloud_for_octomap_pub_.publish(prev_msg_->data.empty() ? msg : prev_msg_);
    state_request = PCL_STATE_CONTROL::RESUME;
  }
  else
  {
    // merge the current msg with previous messages published till now
    // tutorial available at
    // http://www.pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_prev_msg(new pcl::PointCloud<pcl::PointXYZ>);
    convertROStoPCL(prev_msg_, pcl_prev_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

    source = pcl_msg;
    target = pcl_prev_msg;
    PointCloud::Ptr temp(new PointCloud);
    // check if the cloud size is growing exceptionally high. if yes, downsample the pointcloud without impacting
    // objects and features
    pairAlign(source, target, temp, pairTransform);

    // transform current pair into the global transform
    pcl::transformPointCloud(*temp, *result, GlobalTransform);

    // update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

    //          Disabling voxel filter -- enabling this will impact rover detection
    //        float leafsize  = 0.03;
    //        pcl::VoxelGrid<PointT> grid;
    //        grid.setLeafSize (leafsize, leafsize, leafsize);
    //        grid.setInputCloud (result);
    //        grid.filter (*result);

    // clip the point cloud in x y and z direction
    clipPointCloud(result);

    ROS_INFO("PeriodicSnapshotter::mergeClouds : PC size : %d", result->size());

    convertPCLtoROS(result, merged_cloud);
  }

  // publish the merged message
  prev_msg_ = merged_cloud;
  registered_pointcloud_pub_.publish(merged_cloud);
}

void PeriodicSnapshotter::clipPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  pcl::PassThrough<pcl::PointXYZ> globalPassThroughFilter;

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("z");
  globalPassThroughFilter.setFilterLimits(filter_min_z, filter_max_z);
  globalPassThroughFilter.filter(*input_cloud);

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("y");
  globalPassThroughFilter.setFilterLimits(filter_min_y, filter_max_y);
  globalPassThroughFilter.filter(*input_cloud);

  globalPassThroughFilter.setInputCloud(input_cloud);
  globalPassThroughFilter.setFilterFieldName("x");
  globalPassThroughFilter.setFilterLimits(filter_min_x, filter_max_x);
  globalPassThroughFilter.filter(*input_cloud);
}

int main(int argc, char** argv)
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
