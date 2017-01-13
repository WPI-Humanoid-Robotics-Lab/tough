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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <val_common/val_common_names.h>
#include <tf/transform_listener.h>


namespace laser_assembler
{
/**
 * @brief The PeriodicSnapshotter class This class publishes assembled pointcloud after every 2.5 sec. First message is published after around 8sec.
 */
class PeriodicSnapshotter
{

public:
    PeriodicSnapshotter();
    /**
     * @brief cloud_ latest available pointcloud in pcl format
     */
    static pcl::PointCloud<pcl::PointXYZ> cloud_;
//    /**
//     * @brief getNearestPoint Provides mean of nearest points from lidar cloud for a given point.
//     * @param point PointStamped for which nearest point is to be fetched. This variable is updated with the results.
//     * @param K Number of points to be searched near the given point. default value is 1.
//     * @return  true is a point was found false otherwise.
//     */
//    static bool getNearestPoint(geometry_msgs::PointStamped &point, int K);

private:
    void timerCallback(const ros::TimerEvent& e);
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::ServiceClient client_;
    ros::Timer timer_;
    bool first_time_;
} ;


// Initialize static variable.
pcl::PointCloud<pcl::PointXYZ>  PeriodicSnapshotter::cloud_;

}


#endif // PERIODIC_SNAPSHOTTER_H
