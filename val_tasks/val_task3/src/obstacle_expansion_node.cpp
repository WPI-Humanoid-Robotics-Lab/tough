//
// Created by will on 6/14/17.
//
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>

#define EXPAND_DISTANCE 0.2
#define N_INTERVALS 8

ros::Publisher pub;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    // don't tell anyone I said this was a good way to do this
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    out_cloud->header = cloud->header;

    out_cloud->reserve(cloud->size() * N_INTERVALS);

    for (int interval = 0; interval < N_INTERVALS; interval++) {
        float dx = EXPAND_DISTANCE * std::sin(M_PI * 2 / N_INTERVALS);
        float dy = EXPAND_DISTANCE * std::cos(M_PI * 2 / N_INTERVALS);

        for (pcl::PointXYZ pt : *cloud) {
            pt.x += dx;
            pt.y += dy;
            out_cloud->push_back(pt);
        }
    }

    pub.publish(out_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_filter_node");

    ros::NodeHandle nh;

    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/block_map", 10);
    ros::Subscriber sub = nh.subscribe("/field/collision_map_blocker/no_outliers", 10, &callback);

    ros::spin();

    return 0;
}