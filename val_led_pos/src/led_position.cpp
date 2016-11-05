/*
  * NASA's Space Robotics Challenge
  * Qualification Task 1 - Detection and estimation of position and color of LEDs on panel
  * Authors: Ayush Shah, Bhawna Shiwani
*/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
std::vector<std::vector<cv::Point> > contours;
cv::Mat old_image;
bool flag = 0;
cv::RNG rng(12345);

class LED_Detector
{
  //Initialization of ROS subscriber, publisher and node
  ros::NodeHandle node_handle;
  ros::Subscriber image_sub;
  //ros::Publisher image_rgbpub;
  //ros::Publisher image_xyzpub;


  public:
  //Constructor
  LED_Detector(int argc, char** argv)
  {
    image_sub = node_handle.subscribe("/multisense/camera/left/image_raw", 1, &LED_Detector::imageCallback, this);
    //image_rgbpub = node_handle.publisher("/detect/light/rgb", 1);
    //image_xyzpub = node_handle.publisher("/detect/light/xyz", 1);
    cv::namedWindow("Raw Image with Contours");
  }

  //Destructor
  ~LED_Detector()
  {
    cv::destroyWindow("Raw Image with Contours");
  }

  // Callback function
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_depthptr;

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
          old_image = cv_depthptr->image; 
          flag++;
        }
        DetectLED(cv_depthptr->image);
        //image_pub.publish(cv_depthptr->toImageMsg());
      }
  }

  /*
    * This algorithm takes the old frame and subtracts it from the current frame.
    * The difference frame contains noise, so we erode it to get rid of noise and
    * also convert it to grayscale and threshold it. We then find it's countours
    * along with the centroid of the shape. Using the centroid we obtain

  */


  // LED detection algorithm
  void DetectLED(cv::Mat new_image)
  {
    cv::Mat diff, erod, erod_gray, erod_gray_thresh;
    //std::vector<cv::Point2f>points;

    // Finding the difference image
    diff = abs(new_image - old_image);
    //cv::imshow("Difference Image", diff);
      
    // Eroding to the difference in image to get rid of noise
    cv::erode(diff, erod, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    //cv::imshow("Eroded Image", erod);
      
    // Converting difference image to grayscale
    cv::cvtColor(erod, erod_gray, CV_BGR2GRAY);

    // Binary thresholding of grayscale difference image
    cv::threshold(erod_gray, erod_gray_thresh, 40, 255, CV_THRESH_BINARY);
    //cv::imshow("Thresholded Image", erod_gray_thresh);

    // Find the contours
    // Use if condition only to display
    if (countNonZero(erod_gray_thresh) > 1)
    {
        cv::Mat contourOutput = erod_gray_thresh.clone();
        cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    }

    std::vector<std::vector<cv::Point> >contours_poly(contours.size());
    std::vector<cv::Rect>boundRect(contours.size());
    std::vector<cv::Point2f>center(contours.size());

    for( int i = 0; i< contours.size(); i++ )
    {
        approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
    }

    // Approximate contours to polygons + get bounding rect
    // Draw polygonal contour + bounding rects
    // cv::Mat drawing = cv::Mat::zeros(new_image.size(), CV_8UC3);
    for( int i = 0; i< contours.size(); i++ )
    { 
        cv::Moments moment = cv::moments((cv::Mat)contours[i]);
        if (moment.m00)
        {
            points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
            std::cout<<"x:"<<points[j].x<<" y:"<<points[j].y<<"\n";
            std::cout<<new_image.at<cv::Vec3b>(points[j].y,points[j].x)<<"\n";
            j++; 
        }        
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        //boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
        drawContours(new_image, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        rectangle(new_image, boundRect[i].tl(), boundRect[i].br(), color, 1, 8, 0 );
    }

    cv::imshow("Raw Image with Contours", new_image);
    cv::waitKey(3);
    old_image = new_image;

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LED_Detector");
  LED_Detector kc(argc, argv);
  ros::spin();
  return 0;
}