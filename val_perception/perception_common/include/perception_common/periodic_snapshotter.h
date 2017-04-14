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

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>
#include <val_common/val_common_names.h>

namespace laser_assembler
{

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
     * in the launch file. example: <param name="/val_laser_assembler_svc/laser_snapshot_timeout" type="double" value="6.0"/>
     * @param e
     */
    void timerCallback(const ros::TimerEvent& e);

    /**
     * @brief mergeClouds merges the pointcloud published on assembled_cloud2 topic with the previous messages
     * that were published on the same topic
     * @param msg
     */
    void mergeClouds(const sensor_msgs::PointCloud2::Ptr msg);

    void pairAlign (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample_ = true);

    /**
     * @brief getNearestPoint returns the K-nearest neighbours of a specified point. This is a
     * static function and can be accessed directly by using laser_assembler::PeriodicSnapshotter::getNearestPoint format.
     * @param point This is a point with frame_id and timestamp
     * @param K Number of neighbors to search
     * @return true if neighbours are found.
     */
    static bool getNearestPoint(geometry_msgs::PointStamped &point, int K=1);

    /**
     * @brief POINTCLOUD_STATIC_PTR Current snapshot of the pointcloud
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr POINTCLOUD_STATIC_PTR;

private:
    ros::NodeHandle n_;
    ros::Publisher snapshot_pub_;
    ros::Publisher registered_pointcloud_pub_;
    ros::Subscriber snapshot_sub_;
    ros::ServiceClient client_;
    ros::Timer timer_;
    sensor_msgs::PointCloud2::Ptr prev_msg_;
    bool first_time_;
    bool downsample_;


} ;

// always provide type before initializing a static member
pcl::PointCloud<pcl::PointXYZ>::Ptr laser_assembler::PeriodicSnapshotter::POINTCLOUD_STATIC_PTR(new pcl::PointCloud<pcl::PointXYZ>);


// This function should move out to perception common as a service.
bool PeriodicSnapshotter::getNearestPoint(geometry_msgs::PointStamped &point, int K)
{
    if (PeriodicSnapshotter::POINTCLOUD_STATIC_PTR->empty()){
        //        ROS_INFO("Point cloud is empty");
        return false;
    }

    // store the frameID of original point so that we can retransform the output to that frame
    std::string originalFrame = point.header.frame_id;
    point.header.stamp = ros::Time(0);
    tf::TransformListener listener;

    //     transform the point to world frame
    if (originalFrame != VAL_COMMON_NAMES::WORLD_TF){
        try{
            listener.waitForTransform(VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF, VAL_COMMON_NAMES::WORLD_TF, ros::Time(0), ros::Duration(3));
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF,point, point);
        }
        catch(tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            return false;
        }
    }

    // get a kdtree for searching point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (PeriodicSnapshotter::POINTCLOUD_STATIC_PTR);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    // index of points in the pointcloud
    std::vector<int> pointIdxNKNSearch(K);
    //squared distance of points
    std::vector<float> pointNKNSquaredDistance(K);

    // convert input point into PCL point for searching
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.point.x;
    searchPoint.y = point.point.y;
    searchPoint.z = point.point.z;


    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    float meanX=0.0, meanY=0.0, meanZ=0.0;

    //perform nearestKsearch
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
            meanX += cloud->points[ pointIdxNKNSearch[i] ].x;
            meanY += cloud->points[ pointIdxNKNSearch[i] ].y;
            meanZ += cloud->points[ pointIdxNKNSearch[i] ].z;

            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
        point.point.x = meanX = meanX/pointIdxNKNSearch.size();
        point.point.y = meanY = meanY/pointIdxNKNSearch.size();
        point.point.z = meanZ = meanZ/pointIdxNKNSearch.size();

    }

    point.header.stamp = ros::Time(0);

    //transform the point back to its original frame, if required
    if (originalFrame != VAL_COMMON_NAMES::WORLD_TF){
        try{
            listener.transformPoint(originalFrame, point, point);
        }
        catch(tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            return false;
        }
    }

    return true;
}

void convertROStoPCL(const sensor_msgs::PointCloud2::Ptr ros_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_msg){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*ros_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*pcl_msg);

}

void convertPCLtoROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg, sensor_msgs::PointCloud2::Ptr &ros_msg ){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*pcl_msg, pcl_pc2);
    pcl_conversions::moveFromPCL(pcl_pc2, *ros_msg);

}

}


#endif // PERIODIC_SNAPSHOTTER_H
