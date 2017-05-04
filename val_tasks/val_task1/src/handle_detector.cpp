#include <val_task1/handle_detector.h>
#include <visualization_msgs/Marker.h>
#include "val_common/val_common_names.h"

#define SHOWIMAGE true

void handle_detector::showImage(cv::Mat image)
{
#ifdef SHOWIMAGE
  return;
#endif
  cv::namedWindow( "Handle Detection", cv::WINDOW_AUTOSIZE );
  cv::imshow( "Handle Detection", image);
  cv::waitKey(0);
}

void handle_detector::colorSegment(const cv::Mat &imgHSV, const int hsv[6], cv::Mat &outImg)
{
  cv::inRange(imgHSV,cv::Scalar(hsv[0],hsv[2],hsv[4]), cv::Scalar(hsv[1],hsv[3],hsv[5]),outImg);
}

void handle_detector::doMorphology (cv::Mat &image)
{
  cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(15,15)));
  cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
}


void handle_detector::findMaxContour(const cv::Mat image, cv::Rect &roi)
{
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  // Detect edges using canny
  cv::Canny(image, canny_output, thresh_, thresh_*2, 3);

  // Find contours
  cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Approximate contours to polygons + get bounding rects
  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());
  std::vector<std::vector<cv::Point>> hull(contours.size());

  for(int i = 0; i < contours.size(); i++)
  {
    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    cv::approxPolyDP(cv::Mat(hull[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
  }

  int largest_area=0;
  int largest_contour_index=0;
  cv::RNG rng(12345);
  cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

  cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
  for( int i = 0; i< contours.size(); i++ )
  {
    double area = contourArea( hull[i] );  //  Find the area of contour
    if( area > largest_area )
    {
      largest_area = area;
      largest_contour_index = i;               //Store the index of largest contour
    }
  }

  cv::drawContours( drawing, contours, largest_contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
  cv::rectangle( drawing, boundRect[largest_contour_index].tl(), boundRect[largest_contour_index].br(), color, 2, 8, 0 );
  showImage(drawing);
  roi = boundRect[largest_contour_index];
}

bool handle_detector::findAllContours (const cv::Mat &image)
{
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Rect roi;

  // Detect edges using canny
  cv::Canny(image, canny_output, thresh_, thresh_*2, 3);

  // Find contours
  cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Approximate contours to polygons + get bounding rects
  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());
  std::vector<std::vector<cv::Point>> hull(contours.size());
  cv::RNG rng(12345);
  cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
  cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

  for(int i = 0; i < contours.size(); i++)
  {
    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    cv::approxPolyDP(cv::Mat(hull[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
  }

  cv::Point prev_point;
  for( int i = 0; i< contours.size(); i++ )
  {
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
    cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    cv::Point currentPoint = cv::Point((boundRect[i].x + boundRect[i].width*0.5), (boundRect[i].y + boundRect[i].height*0.5));
    if(abs(currentPoint.x - prev_point.x) < 6 && abs(currentPoint.y - prev_point.y) < 6)
      continue;
    rectCenter.push_back(currentPoint);
    std::cout<<rectCenter.back().x<<" "<<rectCenter.back().y<<std::endl;
    prev_point = currentPoint;
  }
  showImage(drawing);

  return false;
}

bool handle_detector::getHandleLocation(std::vector<geometry_msgs::Point>& handleLocs)
{

  bool foundButton = false;
  pcl::PointXYZRGB pclPoint;
  src_perception::StereoPointCloudColor::Ptr organizedCloud(new src_perception::StereoPointCloudColor);
  src_perception::PointCloudHelper::generateOrganizedRGBDCloud(current_disparity_, current_image_, qMatrix_, organizedCloud);
  tf::TransformListener listener;
  tf::Transform transform;
  tf::Quaternion q;
  static tf::TransformBroadcaster br;

  try
  {
    listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::spinOnce();
    return false;
  }

  for(int i=0; i<rectCenter.size(); i++)
  {
    for(int k = -10; k<10; k++)
    {
      pclPoint = organizedCloud->at(rectCenter[i].x, rectCenter[i].y);
      //wtf
      if (pclPoint.z<0)
      {
        rectCenter[i].x+=k;
        rectCenter[i].y+=abs(k);
      }
    }

    std::stringstream ss;
    ss << side_ << "HandleFrame" << frameID_;

    transform.setOrigin( tf::Vector3(pclPoint.x , pclPoint.y, pclPoint.z));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ss.str()));

    frameID_++;
    buttonCenters_.push_back(pclPoint);

    std::cout<<"Handle "<< i << " at " << buttonCenters_[i].x << " " << buttonCenters_[i].y << " " << buttonCenters_[i].z << std::endl;
    geometry_msgs::Point geom_point;
    geom_point.x = pclPoint.x;
    geom_point.y = pclPoint.y;
    geom_point.z = pclPoint.z;

    //ignore points with z less than 0.6m
    if(geom_point.z > 0.6){
      handleLocs.push_back(geom_point);
    }

    // return true only if there are 4 points in handleLocs
    foundButton = true;

  }

  return foundButton;

}

void handle_detector::getReducedImage(cv::Mat &image, const cv::Rect &roi)
{
  image = image(roi);
}

bool handle_detector::findHandles(std::vector<geometry_msgs::Point>& handleLocs)
{

  // clear the vector before appending
  handleLocs.clear();

  ms_sensor_.giveImage(current_image_);
  ms_sensor_.giveDisparityImage(current_disparity_);

  cv::cvtColor(current_image_, current_image_HSV_, cv::COLOR_BGR2HSV);
  cv::GaussianBlur(current_image_HSV_, current_image_HSV_, cv::Size(9, 9), 2, 2);

  side_ = "left"; // used for naming frames. Left = red;

  colorSegment(current_image_HSV_, hsvRed_, imRed_);
  doMorphology(imRed_);
  showImage(imRed_);
  findMaxContour(imRed_, roiRed_);
  imRedReduced_= cv::Mat::zeros(current_image_HSV_.size(), current_image_HSV_.type());

  cv::Mat mask = cv::Mat::zeros(current_image_HSV_.size(), current_image_HSV_.type());
  cv::rectangle(mask, cv::Point(roiRed_.x, roiRed_.y), cv::Point(roiRed_.x+roiRed_.width, roiRed_.y+roiRed_.height),cv::Scalar(255, 255, 255), -1, 8, 0);
  showImage(mask);
  current_image_HSV_.copyTo(imRedReduced_,mask);

  showImage(imRedReduced_);
  colorSegment(imRedReduced_, hsvGray_, imGray_);
  showImage(imGray_);
  showImage(current_disparity_);
  bool val = findAllContours(imGray_);

  side_ = "right"; // used for naming frames;

  colorSegment(current_image_HSV_, hsvBlue_, imBlue_);
  doMorphology(imBlue_);
  showImage(imBlue_);
  findMaxContour(imBlue_, roiBlue_);
  imBlueReduced_= cv::Mat::zeros(current_image_HSV_.size(), current_image_HSV_.type());

  cv::Mat maskBlue = cv::Mat::zeros(current_image_HSV_.size(), current_image_HSV_.type());
  cv::rectangle(maskBlue, cv::Point(roiBlue_.x, roiBlue_.y), cv::Point(roiBlue_.x+roiBlue_.width, roiBlue_.y+roiBlue_.height),cv::Scalar(255, 255, 255), -1, 8, 0);
  showImage(maskBlue);
  current_image_HSV_.copyTo(imBlueReduced_,maskBlue);

  showImage(imBlueReduced_);
  colorSegment(imBlueReduced_, hsvGray_, imGray_);
  showImage(imGray_);
  showImage(current_disparity_);
  val = findAllContours(imGray_);

  return getHandleLocation(handleLocs);
}

handle_detector::handle_detector(ros::NodeHandle nh) : nh_(nh), ms_sensor_(nh_)
{
  ms_sensor_.giveQMatrix(qMatrix_);
}
