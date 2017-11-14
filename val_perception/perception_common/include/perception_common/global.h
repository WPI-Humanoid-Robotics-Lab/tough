/**
 ********************************************************************************************************
 * @file 		global.h
 * @brief		includes and marcos for perception common
 * @details 	Has the general includes for opencv, ros and pcl. Also defines some macros.
 ********************************************************************************************************
 */
#ifndef GLOBAL_H_
#define GLOBAL_H_

/*** INCLUDE FILES ***/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

/*** MACROS ***/
#define TIME(tclock, function, name)\
	tclock=clock();\
	function;\
	std::cout<<"Time taken by "<<name<<" : "<<(double)(clock() - tclock)/CLOCKS_PER_SEC<<std::endl;

#undef DISALLOW_COPY_AND_ASSIGN
//this macro is to make a class not copyable
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete; \
  void operator=(const TypeName&) = delete;

/*** TYPE DEFS ***/

namespace tough_perception
{
	typedef pcl::PointXYZI						LaserPoint;
	typedef	pcl::PointXYZ						StereoPoint;
	typedef	pcl::PointXYZRGB					StereoPointColor;
	typedef pcl::PointCloud<LaserPoint>		 	LaserPointCloud;
	typedef pcl::PointCloud<StereoPoint> 		StereoPointCloud;
	typedef pcl::PointCloud<StereoPointColor> 	StereoPointCloudColor;
}

#endif /* GLOBAL_H_ */
