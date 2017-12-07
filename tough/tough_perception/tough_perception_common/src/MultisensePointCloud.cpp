/**
 ********************************************************************************************************
 * @file    MultisensePointCloud.cpp
 * @brief   MultisensePointCkloud class defnition
 * @details Use to retrive the point clouds from the multisense head
 ********************************************************************************************************
 */

#include <tough_perception_common/MultisensePointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tough_perception_common/perception_common_names.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>


namespace tough_perception {

bool MultisensePointCloud::laser_wrt_l_foot_callback_active_ = false;
bool MultisensePointCloud::laser_callback_active_=false;
bool MultisensePointCloud::stereo_callback_active_=false;

using namespace std;
using namespace pcl;

//Nandan's point structure, used here only to avoid all the issues with PointCloud2 conversion
//maybe inefficient, until we come up with a plan
typedef struct
{
    float x, y, z;
    uint32_t intensity;
} LaserScanPoint;


/**
 * @note constructor
 */
MultisensePointCloud::MultisensePointCloud(ros::NodeHandle &nh, const string base_frame, const string left_camera_opt_frame_tf):
    nh_(nh), base_frame_(base_frame), left_camera_opt_frame_tf_(left_camera_opt_frame_tf),
    laser_cloud_wrt_l_foot_(new LaserPointCloud),
    laser_cloud_(new LaserPointCloud),
    stereo_cloud_(new StereoPointCloud),
    unified_cloud_(new UnifiedPointCloud)
{
	ros::NodeHandle pnh("~");
	if(!pnh.getParam("laser_topic",laser_topic_))
	{
        laser_topic_=PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC;
	}
	if(!pnh.getParam("stereo_topic",stereo_topic_))
	{
        stereo_topic_=PERCEPTION_COMMON_NAMES::MULTISENSE_STEREO_CLOUD_TOPIC;
	}


	new_laser_=false;
    new_laser_wrt_l_foot_=false;
	new_stereo_=false;
	new_unified_=false;
	spindle_rate_=0.00;

}

void MultisensePointCloud::setLaserTopic(const std::string &name)
{
    laser_topic_=name;
}

/**
 * @note TODO: Clean up this entire function
 */
void MultisensePointCloud::saveToLaserCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	//pcl header issues we need to fix this later
	laser_cloud_->header.frame_id   = msg->header.frame_id;
	laser_cloud_->header.seq	= msg->header.seq;
	laser_cloud_->header.stamp = msg->header.stamp.toNSec()/10e2; //the stamp in pcl data type is in ms and not ns
	laser_cloud_->width    = msg->width;
	laser_cloud_->height   = msg->height;
	laser_cloud_->is_dense = msg->is_dense == 1;
    // Allot memory
    uint32_t num_points = msg->width * msg->height;
	laser_cloud_->points.resize (num_points);

	for(size_t bin=0;bin<msg->height;bin++)
	{
		for(size_t i=0;i<msg->width;i++)
		{
			LaserScanPoint *pointT = (LaserScanPoint*) &msg->data[bin * msg->width * sizeof(LaserScanPoint) + i * sizeof(LaserScanPoint)];
			LaserPoint T;
			T.x = pointT->x;
			T.y = pointT->y;
			T.z = pointT->z;
			T.intensity = static_cast<float>(pointT->intensity);
			//Use 30 cm if you onlu want to remove the point clouds
			//Later implement a propoer robot body remover
//			if((T.x*T.x+T.y*T.y+T.z*T.z)<0.30*0.30)
//				T.x=T.y=T.z=T.intensity=-1;
			// fix: if the first line doesn't work, then use the second line
            memcpy (&laser_cloud_->points[bin*msg->width + i], &T, sizeof(tough_perception::LaserPoint));
		}
	}
}

/**
 * @note TODO: Clean up this entire function
 */
void MultisensePointCloud::saveToLaserCloudWrtLFoot(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    //pcl header issues we need to fix this later
    static tf::TransformListener    tf_listener;

    tf_listener.waitForTransform( base_frame_, msg->header.frame_id, ros::Time::now(), ros::Duration(3.0));


    laser_cloud_wrt_l_foot_->header.frame_id =  base_frame_;
    laser_cloud_wrt_l_foot_->header.seq	= msg->header.seq;
    laser_cloud_wrt_l_foot_->header.stamp = msg->header.stamp.toNSec()/10e2; //the stamp in pcl data type is in ms and not ns
    laser_cloud_wrt_l_foot_->width    = msg->width;
    laser_cloud_wrt_l_foot_->height   = msg->height;
    laser_cloud_wrt_l_foot_->is_dense = msg->is_dense == 1;
    // Allot memory
    uint32_t num_points = msg->width * msg->height;
    laser_cloud_wrt_l_foot_->points.resize (num_points);

    for(size_t bin=0;bin<msg->height;bin++)
    {
        for(size_t i=0;i<msg->width;i++)
        {
            LaserScanPoint *pointT = (LaserScanPoint*) &msg->data[bin * msg->width * sizeof(LaserScanPoint) + i * sizeof(LaserScanPoint)];
            LaserPoint T;
            geometry_msgs::PoseStamped  p1, p2;

            p1.pose.position.x = pointT->x;
            p1.pose.position.y = pointT->y;
            p1.pose.position.z = pointT->z;
            p1.pose.orientation.w = 1.0;

            tf_listener.transformPose( base_frame_, p1, p2);

            T.x = p2.pose.position.x;
            T.y = p2.pose.position.y;
            T.z = p2.pose.position.z;

            T.intensity = static_cast<float>(pointT->intensity);
            //Use 30 cm if you onlu want to remove the point clouds
            //Later implement a propoer robot body remover
//			if((T.x*T.x+T.y*T.y+T.z*T.z)<0.30*0.30)
//				T.x=T.y=T.z=T.intensity=-1;
            // fix: if the first line doesn't work, then use the second line
            memcpy (&laser_cloud_wrt_l_foot_->points[bin*msg->width + i], &T, sizeof(tough_perception::LaserPoint));
        }
    }
}

/**
 * @note laser callback
 */
void MultisensePointCloud::laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{


	if(msg->fields[3].datatype==sensor_msgs::PointField::UINT32)
	{
		saveToLaserCloud(msg);
	}
	else
    {
		pcl::PCLPointCloud2 pcl_pc2;
    	pcl_conversions::toPCL(*msg, pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud_);
	}
	//TOFIX: WHAT IS WRONG WITH THIS FUNCTION???
	//WHY DOES IT EAT HALF MY CLOUD??? HELPPP!!!
	//saveToLaserCloud(msg);

	new_laser_=true;
	ROS_INFO_ONCE("Laser Cloud: width = %d, height = %d, header = %s, isdense = %d\n", laser_cloud_->width, laser_cloud_->height,laser_cloud_->header.frame_id.c_str(),laser_cloud_->is_dense);
}

/**
 * @note laser callback
 */
void MultisensePointCloud::laserCallbackWrtLFoot(const sensor_msgs::PointCloud2ConstPtr& msg)
{

    if(msg->fields[3].datatype==sensor_msgs::PointField::UINT32)
    {
        saveToLaserCloudWrtLFoot(msg);
    }
    else
    {
        tf::StampedTransform stamped_tf;

        tf_listener.waitForTransform( base_frame_, left_camera_opt_frame_tf_, ros::Time::now(), ros::Duration(3.0));
        tf_listener.lookupTransform( base_frame_, left_camera_opt_frame_tf_, ros::Time(0), stamped_tf);

        pcl::PCLPointCloud2 pcl_pc2;

        for(size_t bin=0; bin<msg->height; bin++)
        {
            for(size_t i=0; i<msg->width; i++)
            {
                LaserScanPoint *pointT = (LaserScanPoint*) &msg->data[bin * msg->width * sizeof(LaserScanPoint) + i * sizeof(LaserScanPoint)];
                geometry_msgs::PoseStamped  p1, p2;

                p1.header.frame_id = left_camera_opt_frame_tf_;
                p1.pose.position.x = pointT->x;
                p1.pose.position.y = pointT->y;
                p1.pose.position.z = pointT->z;
                p1.pose.orientation.w = 1.0;

                tf_listener.transformPose( base_frame_, p1, p2);

                pointT->x = p2.pose.position.x;
                pointT->y = p2.pose.position.y;
                pointT->z = p2.pose.position.z;
            }
        }

        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud_wrt_l_foot_);

        laser_cloud_wrt_l_foot_->header.frame_id =  base_frame_;
    }
    new_laser_wrt_l_foot_=true;
    ROS_INFO_ONCE("Laser Cloud Wrt LFoot: width = %d, height = %d, header = %s, isdense = %d\n", laser_cloud_->width, laser_cloud_->height,laser_cloud_->header.frame_id.c_str(),laser_cloud_->is_dense);
}
bool MultisensePointCloud::giveLaserCloudForTime(const ros::Time &time, LaserPointCloud::Ptr &out)
{
	if(!laser_callback_active_)
	{
            laser_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(laser_topic_.c_str(), 1, &MultisensePointCloud::laserCallback, this);
			ROS_INFO_STREAM("Listening to laser cloud in: "<<laser_sub_.getTopic()<<endl);
			laser_callback_active_=true;
			ros::Duration(1).sleep();
			ros::spinOnce();
	}
	if(!new_laser_)
	{
		return false;
	}
	//make sure you are subscribing to the point cloud that is assembled by the map
	if(laser_cloud_->header.frame_id!="left_camera_optical_sweep_fixed")
	{
		ROS_ERROR("The point cloud you are using is not subscribed to the map cloud");
		return false;
	}
	tf::StampedTransform stamped_tf;
	try
	{
        tf_listener.waitForTransform(left_camera_opt_frame_tf_,"left_camera_optical_sweep_fixed" , ros::Time(0)/*time ros::Time().fromNSec(laser_cloud_->header.stamp*10e2)*/, ros::Duration(1.0));
        tf_listener.lookupTransform(left_camera_opt_frame_tf_,"left_camera_optical_sweep_fixed" , ros::Time(0)/*time ros::Time().fromNSec(laser_cloud_->header.stamp*10e2)*/, stamped_tf);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		return false;
	}
	Eigen::Affine3d transform;
	tf::transformTFToEigen(stamped_tf,transform);
	out=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud (*laser_cloud_, *out, transform);
	new_laser_=false;
	return true;

}
/**
 * @note stereo callback
 */
void MultisensePointCloud::stereoCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	stereo_cloud_->clear();
	pcl::fromROSMsg(*msg,*stereo_cloud_);
	new_stereo_=true;
	ROS_INFO_ONCE("Stereo Cloud: width = %d, height = %d, header = %s, isdense = %d\n", msg->width, msg->height,stereo_cloud_->header.frame_id.c_str(),stereo_cloud_->is_dense);
}
/**
 * @note TODO: IMPLEMENT IT
 */
bool MultisensePointCloud::giveUnifiedCloud(UnifiedPointCloud::Ptr &out)
{
	if(new_unified_)
	{
		out=unified_cloud_;
		new_stereo_=false;
		return true;
	}
	return(false);
}
/**
 * @note stereo cloud without color
 */
bool MultisensePointCloud::giveStereoCloud(StereoPointCloud::Ptr &out)
{
	if(!stereo_callback_active_)
	{
		stereo_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(stereo_topic_.c_str(), 1, &MultisensePointCloud::stereoCallback, this);;
		ROS_INFO_STREAM("Listening to stereo cloud in: "<<stereo_sub_.getTopic()<<endl);
		stereo_callback_active_=true;
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

	if(new_stereo_)
	{
		out=stereo_cloud_;
		new_stereo_=false;
		return true;
	}
	return(false);
}
/**
 * @note laser cloud with intensity
 */
bool MultisensePointCloud::giveLaserCloud(LaserPointCloud::Ptr &out)
{
	if(!laser_callback_active_)
	{
		laser_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(laser_topic_.c_str(), 1, &MultisensePointCloud::laserCallback, this);
		ROS_INFO_STREAM("Listening to laser cloud in: "<<laser_sub_.getTopic()<<endl);
		laser_callback_active_=true;
		 ros::Duration(1).sleep();
		 ros::spinOnce();
	}
	if(new_laser_)
	{
		out=laser_cloud_;
		new_laser_=false;
		return true;
	}
	return(false);
}

/**
 * @note laser cloud with intensity
 */
bool MultisensePointCloud::giveLaserCloudWrtLFoot(LaserPointCloud::Ptr &out)
{
    if(!laser_wrt_l_foot_callback_active_)
    {
        laser_sub_wrt_l_foot_ = nh_.subscribe<sensor_msgs::PointCloud2>(laser_topic_.c_str(), 1, &MultisensePointCloud::laserCallbackWrtLFoot, this);
        ROS_INFO_STREAM("Listening to laser cloud in: "<<laser_sub_wrt_l_foot_.getTopic()<<endl);
        laser_wrt_l_foot_callback_active_ = true;
         ros::Duration(1).sleep();
         ros::spinOnce();
    }
    if(new_laser_wrt_l_foot_)
    {
        out=laser_cloud_wrt_l_foot_;
        new_laser_wrt_l_foot_ = false;
        return true;
    }
    return(false);
}

/**
 * @note destructor
 */
MultisensePointCloud::~MultisensePointCloud()
{
	laser_sub_.shutdown();
	stereo_sub_.shutdown();

	laser_cloud_.reset();
	stereo_cloud_.reset();
	unified_cloud_.reset();
}

} /* namespace src_perception */
