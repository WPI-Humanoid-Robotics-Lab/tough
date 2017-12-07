#include <tough_perception_common/laser2point_cloud.h>



Laser2PointCloud::Laser2PointCloud(ros::NodeHandle n,
                                   const std::string laserScanTopic,
                                   const std::string baseFrame,
                                   const std::string pointCloudTopic,
                                   const std::string pointCloud2Topic):m_laserScanSubscriber(n.subscribe(laserScanTopic,100, &Laser2PointCloud::scanCallBack, this)) {

    m_pointCloud2Publisher = n.advertise<sensor_msgs::PointCloud2>(pointCloud2Topic, 30);
    m_pointCloudPublisher = n.advertise<sensor_msgs::PointCloud>(pointCloudTopic, 30);
    m_baseFrame.assign(baseFrame);
}

void Laser2PointCloud::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    // Wait till a tranform is available from the frame of scan to the base frame
    if(!this->m_listener.waitForTransform(
                scan_in->header.frame_id,
                m_baseFrame,
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0))){
        return;
    }

    // Got the tf, proceed with conversion
    sensor_msgs::PointCloud cloud;
    m_projector.transformLaserScanToPointCloud(m_baseFrame,*scan_in,
                                               cloud,m_listener);

    // convert pointcloud to pointcloud2
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

    //publish the ros message
    m_pointCloudPublisher.publish(cloud);
    m_pointCloud2Publisher.publish(cloud2);


}


