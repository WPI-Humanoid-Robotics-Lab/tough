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

/** \author Ioan Sucan*/

/**  Modified code written by Ioan Sucan to use for valkyrie*/

#include <cstdio>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/filters/extract_indices.h>

#include "robot_self_filter/self_mask.h"
#include "val_common/val_common_names.h"
#include "perception_common/perception_common_names.h"

class val_self_filter
{
public:

    val_self_filter(void)
    {
        id_          = 1;
        vmPub_       = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
        vmOutputPub_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_FILTERED_CLOUD_TOPIC, 1);
        vmOutputPub2_= nodeHandle_.advertise<sensor_msgs::PointCloud2>(PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_FILTERED_CLOUD_TOPIC2, 1);
        vmSub_       = nodeHandle_.subscribe(PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_CLOUD_TOPIC,100, &val_self_filter::run, this);

        std::vector<robot_self_filter::LinkInfo> links;
        std::string ns = nodeHandle_.getNamespace();
        if (!nodeHandle_.hasParam("robot_self_filter/self_see_links")){
            robot_self_filter::LinkInfo li;
            li.name="base_link";
            li.padding = .05f;
            li.scale = 1.0f;
            links.push_back(li);
        }
        else {
            //get the links to filter out
            std::vector<std::string> ssl_vals;
            nodeHandle_.getParam("robot_self_filter/self_see_links", ssl_vals);

            if(ssl_vals.size() == 0) {
                ROS_WARN("Self see links need to be an array with size >=1");
            }

            for(int i = 0; i < ssl_vals.size(); i++) {
                robot_self_filter::LinkInfo li;
                li.name = ssl_vals.at(i);
                li.padding = 0.05f;
                li.scale = 1.0f;
                links.push_back(li);
            }
        }

        sf_ = new robot_self_filter::SelfMask<pcl::PointXYZ>(tf_, links);
    }

    ~val_self_filter(void)
    {
        delete sf_;
    }

    void run(sensor_msgs::PointCloud::ConstPtr msg_in)
    {
        //do not take a new scan if previous scan is not complete
        static bool isFiltering = false;

        if (!isFiltering){
            isFiltering = true;
            ros::WallTime start = ros::WallTime::now();
            //convert ros message into pcl pointcloud
            sensor_msgs::PointCloud2 msg;
            sensor_msgs::convertPointCloudToPointCloud2(*msg_in, msg);

            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(msg,pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*cloud_in);

            std::vector<int> mask;

            // all the magic happens in next line
            sf_->maskContainment(*cloud_in, mask);

            pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
            outliers->header = cloud_in->header;

            for (unsigned int i = 0 ; i < mask.size() ; ++i)
            {
                // Get indices of all outliers
                if (mask[i] == robot_self_filter::INSIDE )
                {
                    outliers->indices.insert(outliers->indices.end(),i);
                }
            }

            // Remove outliers from the
            subtractPointClouds(cloud_in, outliers);

            sensor_msgs::PointCloud2 cloud2;
            pcl::toPCLPointCloud2(*cloud_in, pcl_pc2);
            pcl_conversions::moveFromPCL(pcl_pc2, cloud2);
            cloud2.header.frame_id.assign(cloud_in->header.frame_id);
            cloud2.header.stamp = ros::Time(pcl_pc2.header.stamp);
            vmOutputPub2_.publish(cloud2);

            sensor_msgs::PointCloud  cloud;
            sensor_msgs::convertPointCloud2ToPointCloud(cloud2,cloud);
            cloud.header = msg_in->header;
            ROS_DEBUG("Took %0.4f seconds for filtering",(ros::WallTime::now() - start).toSec());
            vmOutputPub_.publish(cloud);

            isFiltering = false;
        }

    }

    void subtractPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices::Ptr outliers){
        pcl::ExtractIndices<pcl::PointXYZ> extract ;
        extract.setInputCloud(full_cloud);
        extract.setIndices(outliers);
        extract.setNegative (true);
        extract.filter (*full_cloud);
        return;
    }

protected:

    double uniform(double magnitude)
    {
        return (2.0 * drand48() - 1.0) * magnitude;
    }

    tf::TransformListener                           tf_;
    robot_self_filter::SelfMask<pcl::PointXYZ>      *sf_;
    ros::Publisher                                  vmPub_;
    ros::Publisher                                  vmOutputPub_;
    ros::Publisher                                  vmOutputPub2_;
    ros::Subscriber                                 vmSub_;
    ros::NodeHandle                                 nodeHandle_;
    pcl::PointCloud<pcl::PointXYZ>                  maskCloud_;

    int                                             id_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "val_self_filter");

    val_self_filter t;
    ros::spin();
    //    t.run();

    return 0;
}
