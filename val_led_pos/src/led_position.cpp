#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
static const std::string OPENCV_WINDOW = "Raw Image";
static const std::string OPENCV_WINDOW_1 = "LED Detection";
std::vector<std::vector<cv::Point> > contours;
cv::Mat old_img;
bool flag = 0;
cv::RNG rng(12345);
class LED_Detector
{
  //Initialization of ROS sub, pub and node
  ros::NodeHandle node_handle;
  ros::Subscriber image_sub;
  //ros::Publisher image_pub;


public:
  //Constructor
  LED_Detector(int argc, char** argv)
  {
    image_sub = node_handle.subscribe("/multisense/camera/left/image_raw", 1, &LED_Detector::imageCallback, this);
    //image_pub = node_handle.publisher("/edge_detector/raw_image", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }
  
  //Destructor
  ~LED_Detector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

// Callback function
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_depthptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_depthptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_depthptr->image.rows > 400 && cv_depthptr->image.cols > 600)
    { 
	if (flag == 0)
	{
		old_img = cv_depthptr->image; 
		flag++;
	}
	DetectLED(cv_depthptr->image);
	//image_pub.publish(cv_depthptr->toImageMsg());
    }
  }




// LED detection algorithm
  void DetectLED(cv::Mat img)
  {


	//Finding the difference image

	cv::Mat diff = abs(img - old_img);

	//Eroding to the difference in image to get rid of noise
	cv::erode(diff, diff, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

	//Converting difference image to grayscale
	cv::cvtColor(diff, diff, CV_BGR2GRAY);

	//Binary thresholding of grayscale difference image
    	cv::threshold(diff, diff, 22, 255, CV_THRESH_BINARY);

	//Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    	//std::vector<std::vector<cv::Point> > contours;

	//Use if condition only to display
	if (countNonZero(diff) > 1) 
	{
    	cv::Mat contourOutput = diff.clone();
    	cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
	}
	/// Approximate contours to polygons + get bounding rects and circles
 	std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
  	//std::vector<cv::Rect> boundRect( contours.size() );
  	std::vector<cv::Point2f>center( contours.size() );
  	//std::vector<float>radius( contours.size() );

  	for( int i = 0; i < contours.size(); i++ )
     		{ approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
       		//boundRect[i] = boundingRect( cv::Mat(contours_poly[i]) );
       		//minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
     		}


  	/// Draw polygonal contour + bonding rects + circles
  	cv::Mat drawing = cv::Mat::zeros( img.size(), CV_8UC3 );
  	for( int i = 0; i< contours.size(); i++ )
     		{
     		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       		drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
       		//rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       		//circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
     		}

    	cv::imshow(OPENCV_WINDOW, drawing);
    	//cv::imshow(OPENCV_WINDOW_1, dst);
    	cv::waitKey(3);
	old_img = img;

  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LED_Detector");
  LED_Detector kc(argc, argv);
  ros::spin();
  return 0;
}
