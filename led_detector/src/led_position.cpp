/*
  * NASA's Space Robotics Challenge
  * Qualification Task 1 - Detection and estimation of position and color of LEDs on panel
  * Authors: Ayush Shah, Bhawna Shiwani, Nathan Denneler, Team M2M (Adhavan, Janani, Praneeta, Shlok)
*/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

namespace enc = sensor_msgs::image_encodings;
std::vector<std::vector<cv::Point> > contours;
bool flag = false;
cv::RNG rng(12345);


class LED_Detector
{
  //Initialization of ROS subscriber, publisher and node
  ros::NodeHandle node_handle;
  ros::Subscriber image_sub;
  ros::Publisher image_rgbpub;
  ros::Publisher image_xyzpub;
  cv_bridge::CvImagePtr cv_depthptr;
  cv::Mat old_image;
  
  public:
  //Constructor
  LED_Detector(int argc, char** argv)
  {
    image_sub = node_handle.subscribe("/multisense/camera/left/image_raw", 1, &LED_Detector::imageCallback, this);
    //pointCloud = node_handle.subscribe("/multisense/image_points2", 1, &LED_Detector::imageCallback, this);
    image_rgbpub = node_handle.advertise<std_msgs::Int32MultiArray>("/detect/light/rgb", 100);
    image_xyzpub = node_handle.advertise<geometry_msgs::Point>("/detect/light/xy", 100);
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
        if (flag == false)
        {
          old_image = cv_depthptr->image; 
          flag = true;
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
          cv::Vec3b pixs_value =  cv_depthptr->image.at<cv::Vec3b>(points[i].y,points[i].x);
          std_msgs::Int32MultiArray rgb;
          geometry_msgs::Point pixelCoordinates;
          pixelCoordinates.x = points[i].x;
          pixelCoordinates.y = points[i].y;
          //pixelCoordinates.z = 0;
          rgb.data.clear();
          for (int iter=2;iter>=0;iter--)
              rgb.data.push_back(pixs_value.val[iter]);
          image_rgbpub.publish(rgb);
          image_xyzpub.publish(pixelCoordinates);
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LED_Detector");
  LED_Detector kc(argc, argv);
  ros::spin();
  return 0;
}