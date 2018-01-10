/**
 ********************************************************************************************************
 * @file test_organizedRGBD.cpp
 * @brief main function to publish rgbd cloud from the multisense 
 * @details this function publishes multisense cloud as an rgbd image, so we can use algorithms specific
 * 			to kinnect, only problem it does not publish calibration files so far
 ********************************************************************************************************
 */

/*** INCLUDE FILES ***/

#include <tough_perception_common/PointCloudHelper.h>
#include <tough_perception_common/MultisenseImage.h>


using namespace tough_perception;
int main(int argc, char** argv)
{
	ros::init(argc,argv,"test_rgbdgen");

	ros::NodeHandle nh;
	MultisenseImage mi(nh);

	StereoPointCloudColor::Ptr organized_cloud(new StereoPointCloudColor);
	cv::Mat color;
	cv::Mat_<float> disp;
	cv::Mat_<double> Q;

	bool valid_Q=false;
	bool new_color=false;
	bool new_disp=false;

	ros::Publisher debug_publisher = nh.advertise<pcl::PCLPointCloud2> ("/debug/RGBD", 1);

	while(ros::ok())
	{
        if(mi.giveImage(color))
		{
			new_color=true;
		}
		if(mi.giveDisparityImage(disp))
		{
			new_disp=true;
		}

		if(new_disp&&new_color)
		{
			if(!mi.giveQMatrix(Q))
			{
				ros::spinOnce();
				continue;
			}
			std::cout<<disp.cols<<" x "<<disp.rows<<std::endl;
			PointCloudHelper::generateOrganizedRGBDCloud(disp,color,Q,organized_cloud);
			ROS_INFO_STREAM("Organized cloud size: "<<organized_cloud->size());
			pcl::PCLPointCloud2 output;
			pcl::toPCLPointCloud2(*organized_cloud,output);
            output.header.frame_id=std::string("world");
			output.header.stamp=ros::Time::now().toNSec();
			debug_publisher.publish(output);

			new_disp=new_color=false;

		}
		ros::spinOnce();
	}

}
