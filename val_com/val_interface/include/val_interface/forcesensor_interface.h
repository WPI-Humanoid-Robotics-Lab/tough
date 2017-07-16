#pragma once

#include <ros/ros.h>
#include <val_hardware_msgs/valAtiSensor.h>
#include <mutex>

class forcesensor {

private:
 
	//node handle
	ros::NodeHandle nh_;
	// subscriber
   	ros::Subscriber sub_force_;

	const std::string forcesensorTopic_;
	val_hardware_msgs::valAtiSensor msg_;
	std::mutex mtx_;
	void forcesensor_callback(const val_hardware_msgs::valAtiSensor& msg_in);
public:
    forcesensor(ros::NodeHandle& nh, const std::string forcesensorTopic);
	~forcesensor();
	//methods
	val_hardware_msgs::valAtiSensor getForce(void);
};

