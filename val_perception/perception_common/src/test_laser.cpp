/**
 ********************************************************************************************************
 * @file    test_laser.cpp
 * @brief   main function to test if the laser cloud is good
 * @details Used to test if laser cloud id being published
 ********************************************************************************************************
 */


#include <ros/ros.h>
#include <perception_common/MultisensePointCloud.h>
#include <pcl/conversions.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"pub_pointcloud");
	ros::NodeHandle nh;
    src_perception::MultisensePointCloud point_cloud_combiner(nh, "/leftFoot", "/left_camera_frame");
	ros::Publisher pub_laser_test = nh.advertise<pcl::PCLPointCloud2> ("/test/laser_points2", 1,true);
	while(ros::ok())
	{
		src_perception::LaserPointCloud::Ptr cloud;
        /*if(point_cloud_combiner.giveLaserCloud(cloud))
		{
			pcl::PCLPointCloud2 output;
			pcl::toPCLPointCloud2(*cloud,output);
//			pcl::PointXYZI pt;
//			memcpy(&pt,&output.data[0],sizeof(float)*4);
//			std::cout<<cloud->points[0].intensity<<"->"<<pt.intensity<<std::endl;
			output.header.stamp=ros::Time::now().toNSec()/10e2;
			pub_laser_test.publish(output);
        }*/

        ROS_INFO_ONCE("meow!");

        if (point_cloud_combiner.giveLaserCloudWrtLFoot(cloud))
        {
            pcl::PCLPointCloud2 output;
            pcl::toPCLPointCloud2(*cloud, output);
            output.header.stamp = ros::Time::now().toNSec()/10e2;
            pub_laser_test.publish(output);
        }

		ros::spinOnce();
	}
	return(0);
}


