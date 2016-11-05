#ifndef FINDBUTTON_H
#define FINDBUTTON_H


#include <iostream>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <assert.h> 

class FindButton
{
public:
	//constructor takes in nodeHandle 
	FindButton(src_perception::MultisenseImage*);  
	// safty function to see if there is a button
	bool buttonDectected();
	//get the location of the button
	geometry_msgs::PointStamped getLocation();
private:
	//glabel node
	src_perception::MultisenseImage *mi;
	//Proccess the image
	geometry_msgs::Point processImage(cv::Mat);
	//gather the nessary Sensor data
	//
};
#endif