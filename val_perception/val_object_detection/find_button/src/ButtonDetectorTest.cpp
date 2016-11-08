

#include <ros/ros.h>
#include <find_button/ButtonDetector.h>
#include <perception_common/MultisenseImage.h>


int main(int argc, char** argv)
 {
	ros::init(argc,argv,"test_button");
	ros::NodeHandle nh;
//	src_perception::MultisenseImage mi(nh);
    ButtonDetector test(nh);
	while(ros::ok())
	{
		ros::spinOnce();
		test.getLocation();
	}


}
