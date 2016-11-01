/**
 ********************************************************************************************************
 * @file    test_MultisenseControl.cpp
 * @brief   the main function to test the multisense Contriol functionality
 * @details This test if the Multisense is switching between the high resolution and low resolution
 ********************************************************************************************************
 */
#include <ros/ros.h>
#include <perception_common/MultisenseControl.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"test_multisense_control");
	ros::NodeHandle nh;
	drc_perception::MultisenseControl nm(nh);

	//switches between low mode and high every 2.5min
	while(ros::ok())
	{
		nm.setHighMode();
		ros::Duration(50).sleep();
		//note that the chnage will take 30 secs
		nm.setLowMode();
		ros::Duration(50).sleep();
	}
}
