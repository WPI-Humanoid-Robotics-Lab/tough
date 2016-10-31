/**
 ********************************************************************************************************
 * @file    MultisensePointCloud.cpp
 * @brief   this class describbes the MultisensePoinTCLoud class
 * @details Used to get the data from the laser and the stereo
 ********************************************************************************************************
 */
 /*
 * The objective of this class is to act as the interface between the multisense ros datatypes with the standard PCL data structures
 * * It ensures that everyone is using the consistent and most efficient method to access the Multisense point clouds
 * * It prevents duplication of effort to write functions to do the same by every other node
 * * Additionally it also gives access to unified point cloud data type
 */

#ifndef MULTISENSEPOINTCLOUD_H_
#define MULTISENSEPOINTCLOUD_H_

#include <perception_common/global.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"

namespace src_perception {

/**
 * TODO: Currently the laser cloud is retured for any call to the unified datatype
 * 		 Once unified datatype is implement replace it.
 */
typedef LaserPointCloud						UnifiedPointCloud;
typedef LaserPoint							UnifiedPoint;

class MultisensePointCloud
{

	DISALLOW_COPY_AND_ASSIGN(MultisensePointCloud)

	ros::NodeHandle 					nh_;

	LaserPointCloud::Ptr 				laser_cloud_;
    LaserPointCloud::Ptr                laser_cloud_wrt_l_foot_;
	StereoPointCloud::Ptr 				stereo_cloud_;
	UnifiedPointCloud::Ptr				unified_cloud_;

	ros::Subscriber 					laser_sub_;
    ros::Subscriber 					laser_sub_wrt_l_foot_;
	ros::Subscriber 					stereo_sub_;

	std::string 						laser_topic_,
										stereo_topic_;

	bool 								new_stereo_;
	bool								new_laser_;    
    bool                                new_laser_wrt_l_foot_;
	bool								new_unified_;

	static bool							laser_callback_active_;
    static bool                         laser_wrt_l_foot_callback_active_;
	static bool							stereo_callback_active_;

    int                                 spindle_rate_;

    tf::TransformListener 				tf_listener;

    /**
     * @brief This function is an internal function which is the actual callback executed when
     *        the laser subscriber is active.
     * @param msg the message received
     */
    void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    /**
     * @brief This function is an internal function which is the actual callback executed when
     *        the laser subscriber is active.
     * @param msg the message received
     */
    void laserCallbackWrtLFoot(const sensor_msgs::PointCloud2ConstPtr& msg);
    /**
     * @brief callback that is executed when the stereo cloud is received
     * @param msg the pointcloud message received
     */
    void stereoCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    /**
     * @brief this function converts from the ros messsage type to point cloud type
     * @param msg the cloud that needs to be converted
     */
	void saveToLaserCloud(const sensor_msgs::PointCloud2ConstPtr &msg);
    void saveToLaserCloudWrtLFoot(const sensor_msgs::PointCloud2ConstPtr &msg);

public:
    /**
     * @brief This function removes the points that correspond to the robot
     *        NOTE: THIS FUNCTION IS DEPRECATED. AS WE ARE FILTERING IN THE LASER CALLBACK
     * @param cloud
     */
    template<class T>
	void removeRobot(pcl::PointCloud<T> &cloud);
    /**
     * @brief the constructor with the node handle.
     * @param nh
     */
	MultisensePointCloud(ros::NodeHandle &nh);
	/**
	 * @brief this function gives the stereo cloud and connects to the subscribers
	 * @param out the pointcloud<pcl::PointXYZ>
	 * @return true if new data is available
	 */
	bool giveStereoCloud(StereoPointCloud::Ptr &out);
	/**
	 * @brief this function gives the laser point cloud
	 * @param out the laser point cloud as PointCloud<pcl::PointXYZ!>
	 * @return	true if new data is available
	 */
	bool giveLaserCloud(LaserPointCloud::Ptr &out);
    /**
     * @brief this function gives the laser point cloud
     * @param out the laser point cloud as PointCloud<pcl::PointXYZ!>
     * @return	true if new data is available
     */
    bool giveLaserCloudWrtLFoot(LaserPointCloud::Ptr &out);
	/**
	 * @brief this function gives the unified point cloud
	 * 		  TODO: Implement it
	 * @param out the unified point cloud
	 * @return
	 */
	bool giveUnifiedCloud(UnifiedPointCloud::Ptr &out);
	virtual ~MultisensePointCloud();

	void setLaserTopic(const std::string &name);
	bool giveLaserCloudForTime(const ros::Time &time, LaserPointCloud::Ptr &out);
};

//might be shifted to a hpp file which has the instantiation of all the templates

template <class T>
void MultisensePointCloud::removeRobot(pcl::PointCloud<T> &cloud)
{
	pcl::PassThrough<T> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.3, 0.3);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.3, 0.3);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-0.3, 0.3);
	pass.setNegative(true);
	pass.filter (*cloud);

}

} /* namespace drc_perception */
#endif
