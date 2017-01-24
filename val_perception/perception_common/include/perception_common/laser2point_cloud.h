#ifndef VAL_LASER2POINT_CLOUD_H
#define VAL_LASER2POINT_CLOUD_H
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

/**
 * \class Laser2PointCloud
 *
 * \brief Convert Laserscan to PointCloud
 *
 * This class subscribes to a topic to read laserscan data and
 * converts it to pointcloud2 message to publish on specified topic.
 * Currently it only publishes message in pointcloud2 format.
 *
 * \author (last to touch it) $Author: Vinayak Jagtap $
 *
 * \version $Revision: 1.0 $
 *
 * \date $Date: 2016/09/28$
 *
 * Contact: vvjagtap@wpi.edu
 *
 */
class Laser2PointCloud{

private:

    /**
     * @brief m_projector class variable used to convert laserscan to pointcloud
     */
    laser_geometry::LaserProjection m_projector;

    /**
     * @brief m_listener class variable used to read tf data
     */
    tf::TransformListener m_listener;

    /**
     * @brief m_pointCloud2Publisher class variable used to publish pointcloud2
     */
    ros::Publisher m_pointCloud2Publisher;

    /**
     * @brief m_pointCloudPublisher class variable used to publish pointcloud
     */
    ros::Publisher m_pointCloudPublisher;

    /**
     * @brief m_laserScanSubscriber class variable used to subscribe to laserscan topic
     */
    ros::Subscriber m_laserScanSubscriber;

    /**
     * @brief m_baseFrame   class variable to store base frame
     */
    std::string m_baseFrame;

    /**
     * @brief scanCallBack Callback function to be called when the laser_scan topic pushes any data
     * @param scan_in   constant pointer to a laserscan message. Don't call this function manually, it is handled by ROS
     */
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in);





public:
    /**
     * @brief Laser2PointCloud uses a provided nodehandler to publish pointcloud message
     * @param n Ros::NodeHandle used for subscribing and publishing topics.
     * @param laserScanTopic an std::string specifying the laser_scan topic to subscribe to.
     * @param baseFrame an std::string specifying the frame that is to be used as a reference for creating pointcloud
     * @param pointCloudTopic an std::string specifying the topic name on which pointcloud2 data will be published
     */
    Laser2PointCloud(ros::NodeHandle n, const std::string laserScanTopic, const std::string baseFrame, const std::string pointCloudTopic);


};



#endif
