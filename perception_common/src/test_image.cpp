/**
 ********************************************************************************************************
 * @file    test_image.cpp
 * @brief   tests if we are avke to get image from the multisense
 * @details Used to test if the images are being published by the Multisense
 ********************************************************************************************************
 */
#include <perception_common/MultisenseImage.h>
#include <perception_common/ImageHelper.h>
#include <perception_common/utils.h>
#include <thread>
//#include <gtest/gtest.h>

TimePlot time_data(40);
Gnuplot *plot_;

void testretriveImageWithoutSpin()
{
	ros::NodeHandle nh;
	drc_perception::MultisenseImage image_assistance(nh);

	cv::Mat disp;
	image_assistance.giveDisparityImage(disp);
	int ctr=0;
	while(ros::ok())
	{
		if(ctr++>1000)
			break;
		image_assistance.giveDisparityImage(disp);
		cv::imshow("Disparity", disp);
		ros::spinOnce();
		cv::waitKey(1);
	}
}

void display()
{
	while(ros::ok())
		time_data.plot(plot_);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"pub_pointcloud");
	//testing::InitGoogleTest(&argc,argv);
	testretriveImageWithoutSpin();
	// ros::NodeHandle nh;
	// drc_perception::MultisenseImage image_assistance(nh);
	// cv::Mat color_image;
	// cv::Mat_<float> disp;
	// cv::namedWindow("Color Image");
	// cv::Mat *disp_img;
	// ros::Time time;
	// ros::Time ptime;

	float camera_fps;

	time_data.add("device fps");
	plot_=new Gnuplot();
	time_data.setup(plot_);

//	int frame_id=0;
//	std::thread thread(display);
//	RUN_ALL_TESTS();

// 	while(ros::ok())
// 	{
// //		if(image_assistance.giveLeftColorImage(color_image))
// //		{
// //			image_assistance.giveTime(time);
// //			cv::imshow("Color Image",color_image);
// //			assert(ptime<time);
// //			ros::Duration d(time-ptime);
// //			camera_fps=1.0/d.toSec();
// //			ptime=time;
// //			time_data.updateValue(frame_id++,"device fps",camera_fps);
// //			cv::waitKey(1);
// //		}
// //		if(image_assistance.giveDisparityImage(disp))
// //		{
// //			drc_perception::ImageHelper::colorDisparity(disp,disp_img);
// //			cv::imshow("Disparity Image",*disp_img);
// //			image_assistance.giveTime(time);
// //			assert(ptime<time);
// //			ros::Duration d(time-ptime);
// //			camera_fps=1.0/d.toSec();
// //			ptime=time;
// //			time_data.updateValue(frame_id++,"device fps",camera_fps);
// //			cv::waitKey(1);
// //		}
// 		if(image_assistance.giveSyncImageswTime(color_image,disp,time))
// 		{
// 			cv::imshow("Color Image",color_image);
// 			drc_perception::ImageHelper::colorDisparity(disp,disp_img);
// 			cv::imshow("Disparity Image",*disp_img);
// 			cv::waitKey(1);
// 			assert(ptime<time);
// 			ros::Duration d(time-ptime);
// 			camera_fps=1.0/d.toSec();
// 			//clearing up the pointers
// 			ptime=time;
// 			time_data.updateValue(frame_id++,"device fps",camera_fps);
// 		}
// 		ros::spinOnce();
// 		//ros::Rate(30).sleep();
// 	}

}



