#ifndef FINDBUTTON_H
#define FINDBUTTON_H


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <assert.h> 

class ButtonDetector
{
public:
	//constructor takes in nodeHandle 
	ButtonDetector(src_perception::MultisenseImage*, ros::NodeHandle& );
	//ButtonDetector(ros::NodeHandle &n);  

	// safty function to see if there is a button
	bool buttonDetected();
	//get the location of the button
	void getLocation(geometry_msgs::Point, cv::Mat);
private:
	//glabel node
	ros::Publisher pubButtonCenter;
	src_perception::MultisenseImage *mi;
	ros::NodeHandle nh;
	geometry_msgs::Point buttonCenter;
	//Proccess the image
	bool processImage();

	//gather the nessary Sensor data
	//
};
#endif