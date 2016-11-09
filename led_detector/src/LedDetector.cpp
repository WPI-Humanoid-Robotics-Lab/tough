#include <led_detector/LedDetector.h>
using namespace cv;
using namespace std;
using namespace src_perception;

namespace src_qual1_task
{
	LedDetector::LedDetector(ros::NodeHandle rosHandle )
		// :mi(new src_perception::MultisenseImage(rosHandle))
	{
		nh = rosHandle;

		multisenseImage = NULL;
		multisensePc = NULL;

		imageRGBpub = node_handle.advertise<std_msgs::Int32MultiArray>("/detect/light/rgb", 100);
		imageXYZpub = node_handle.advertise<geometry_msgs::Point>("/detect/light/xy", 100);
	    // cv::namedWindow("Raw Image with Contours"); //For demo only
	}
	LedDetector::~LedDetector(){
	    // cv::destroyWindow("Raw Image with Contours"); //For demo only
	    // delete mi;
	}


	bool LedDetector::detectLed(){
		// This is the function where we should implement the algorithm
		ImageFrame              img_frame;
		cv::Mat                 img_3d;
		if ((multisenseImage == NULL) || (multisensePc == NULL))
			return false;
		//image_frame.img is the cv image to process
		while(ros::ok()){
			ros::spinOnce();
			if (multisenseImage->giveSyncImages(img_frame.img, img_frame.disp)){
				if (img_frame.Q.empty())
				{
					if (!multisenseImage->giveQMatrix(img_frame.Q))
						continue;
					else
						multisenseImage->giveQMatrix(img_frame.Q);
				}
				cv::reprojectImageTo3D(img_frame.disp, img_3d, img_frame.Q, false); 
    			if (flag == false)
    			{
      				old_image = image_frame.img; 
      				flag = true;
    			}
    			DetectLED(image_frame.img);
    			//image_pub.publish(cv_depthptr->toImageMsg());
				// now we should use ledDetector with image_frame.img
				// I think this also does the job?:
				// pcl::PointXYZRGB point = organized_cloud->at(index.x,index.y);
			}
		}

	}
	void LedDetector::getLedColor(){

	}
	void LedDetector::getLedLocation(){

	}
}
// LED detection algorithm
void DetectLED(const cv::Mat &new_image) // small change here !!!! Since no changes on the image Vinayak suggested passing it as reference
{
	cv::Mat diff, erod, erod_dil, erod_dil_gray, erod_dil_gray_thresh;

	// Finding the difference image
	diff = abs(new_image - old_image);
	//cv::imshow("Difference Image", diff);

	// Get the BGR channels
	std::vector<cv::Mat> channels(3);
	cv::split(diff, channels);

	// Add blue to green(standard conversion to grayscale is .3R + .6G + .1B) which is less than ideal.
	channels[1] = channels[1] + channels[0]/4;
	channels[1] = channels[1] + channels[0]/12;
	cv::merge(channels, diff);

	// Eroding to the difference in image to get rid of noise
	cv::erode(diff, erod, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	//cv::imshow("Eroded Image", erod);

	// Dialate difference, to reflect size of led in the original image.
	cv::dilate(erod, erod_dil, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);  
	//cv::imshow("Eroded and dialated Image", erod_dil);

	// Converting difference image to grayscale
	cv::cvtColor(erod_dil, erod_dil_gray, CV_BGR2GRAY);

	// Binary thresholding of grayscale difference image
	cv::threshold(erod_dil_gray, erod_dil_gray_thresh, 45, 255, CV_THRESH_BINARY);
	cv::imshow("Thresholded Image", erod_dil_gray_thresh);

	// Find the contours
	// Use if condition only to display
	if (countNonZero(erod_dil_gray_thresh) > 1)
	{
		cv::Mat contourOutput = erod_dil_gray_thresh.clone();
		cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	}

	std::vector<std::vector<cv::Point> >contours_poly(contours.size());
	//std::vector<cv::Rect>boundRect(contours.size());
	std::vector<cv::Point2f>center(contours.size());
	std::vector<cv::Point2f>points;

	for( int i = 0; i< contours.size(); i++ )
	{
		approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
	  //boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
	}

	//cv::Mat drawing = cv::Mat::zeros(new_image.size(), CV_8UC3);      //Use this to display the contours on an empty background
	for( int i = 0; i< contours.size(); i++ )
	{   
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours(new_image, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	  //rectangle(new_image, boundRect[i].tl(), boundRect[i].br(), color, 1, 8, 0 );
	}
	for (int i = 0, j = 0; j < contours.size(); j++)
	{
		cv::Moments moment = cv::moments((cv::Mat)contours[j]);
		if (moment.m00)
		{
			points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
			cv::Vec3b pixel_value =  cv_depthptr->image.at<cv::Vec3b>(points[i].y,points[i].x);
			std_msgs::Int32MultiArray rgb;
			geometry_msgs::Point pixelCoordinates;
			pixelCoordinates.x = points[i].x;
			pixelCoordinates.y = points[i].y;
	      //pixelCoordinates.z = 0;
			rgb.data.clear();
			for (int iter=2;iter>=0;iter--)
				rgb.data.push_back(pixel_value.val[iter]);
			imageRGBpub.publish(rgb);
			imageXYZpub.publish(pixelCoordinates);
			i++;
		}
	}
	cv::imshow("Raw Image with Contours", new_image);
	cv::waitKey(3);
	old_image = new_image;
}


	/**
	Function to convert 2D pixel point to 3D point by extracting point
	from PointCloud2 corresponding to input pixel coordinate. This function
	can be used to get the X,Y,Z coordinates
	*/
	void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
	{
		// get width and height of 2D point cloud data
		int width = pCloud.width;
		int height = pCloud.height;

		// Convert from u (column / width), v (row/height) to position in array
		// where X,Y,Z data starts
		int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

		// compute position in array where x,y,z data start
		int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
		int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
		int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

		float X = 0.0;
		float Y = 0.0;
		float Z = 0.0;

		memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
		memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
		memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

		// put data into the point p
		p.x = X;
		p.y = Y;
		p.z = Z;
	}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "led_detection");
    ros::NodeHandle nh;//rosHandle?

    // Eigen::Vector3d                 valve_centre_pt, valve_normal;
    src_qual1_task::LedDetector   ledDetector(nh);

    std::signal(SIGINT, shutdown);

    ledDetector.multisenseImage = new src_perception::MultisenseImage(nh);
    ledDetector.multisensePc = new src_perception::MultisensePointCloud(nh);

    // multisense_image.giveSyncImages(img, disp)
    while (ros::ok())
    {
    	ledDetector.detectLed();

    	ROS_INFO_STREAM("debbuging");

    }

    return 0;
}