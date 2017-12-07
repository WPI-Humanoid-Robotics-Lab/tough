#ifndef FINDBUTTON_H
#define FINDBUTTON_H


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisensePointCloud.h>
#include <tough_perception_common/PointCloudHelper.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <assert.h> 

class ButtonDetector
{
public:
	//constructor takes in nodeHandle 
    ButtonDetector(ros::NodeHandle);
    ~ButtonDetector();
	//ButtonDetector(ros::NodeHandle &n);  

	// safty function to see if there is a button
	bool buttonDetected();
	//get the location of the button
	void getLocation();
private:
	//glabel node
	ros::Publisher pubButtonCenter;
    src_perception::MultisenseImage* mi;
	//geometry_msgs::Point buttonCenter;
	//Proccess the image
    geometry_msgs::Point processImage(const cv::Mat&);

	//gather the nessary Sensor data
	//
};
#endif
