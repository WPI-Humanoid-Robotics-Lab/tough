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

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"

#include "perception_common/periodic_snapshotter.h"

/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */

using namespace laser_assembler ;

PeriodicSnapshotter::PeriodicSnapshotter()
{
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud2", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(5,0), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
}

void PeriodicSnapshotter::timerCallback(const ros::TimerEvent& e)
{

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
        first_time_ = false;
        return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans2 srv;
    srv.request.begin = e.last_real;
    srv.request.end   = e.current_real;

    // Make the service call
    if (client_.call(srv))
    {
        ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.data.size())) ;
        pub_.publish(srv.response.cloud);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(srv.response.cloud,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*laser_assembler::PeriodicSnapshotter::POINTCLOUD_STATIC_PTR);
        ROS_INFO("Size of pointcloud is %d", laser_assembler::PeriodicSnapshotter::POINTCLOUD_STATIC_PTR->size());

    }
    else
    {
        ROS_ERROR("Error making service call\n") ;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "periodic_snapshotter");
    ros::NodeHandle n;
    ROS_INFO("Waiting for [build_cloud] to be advertised");
    ros::service::waitForService("build_cloud");
    ROS_INFO("Found build_cloud! Starting the snapshotter");
    PeriodicSnapshotter snapshotter;
    ros::Rate looprate(1);
    while(ros::ok())
    {
      ros::spinOnce();
      looprate.sleep();
    }
    //ros::spin();

//    ros::spinOnce();
//    while (ros::ok()){
//        geometry_msgs::PointStamped testPoint;
//        testPoint.point.x = 2.9;
//        testPoint.point.y = 1.0;
//        testPoint.point.z = 0.2;
//        testPoint.header.frame_id= VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF;
//        laser_assembler::PeriodicSnapshotter::getNearestPoint(testPoint, 10);
//        ROS_INFO("x:%.2f  y:%.2f  z:%.2f . Size of pointcloud is %d", testPoint.point.x, testPoint.point.y, testPoint.point.z, laser_assembler::PeriodicSnapshotter::POINTCLOUD_STATIC_PTR->size());
//        ros::spinOnce();
//    }
    return 0;
}