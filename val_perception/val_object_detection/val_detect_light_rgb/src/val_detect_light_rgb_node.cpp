#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

class LightDetect{
private:
  ros::Subscriber sub;
  ros::Publisher pub;
  void chatterCallback(const sensor_msgs::ImageConstPtr& img);
  cv_bridge::CvImage z,hist,backP;
  // cv_bridge::CvImage in1;
  int i;
  cv_bridge::CvImagePtr in,out;
public:
  LightDetect(ros::NodeHandle n);

};
LightDetect::LightDetect(ros::NodeHandle n)
{
  i = 1;
  z.encoding = sensor_msgs::image_encodings::BGR8;
  //old frame
  //in1.encoding = sensor_msgs::image_encodings::BGR8;
  pub = n.advertise<sensor_msgs::Image>("/LightDetect",100);
  sub= n.subscribe("/multisense/camera/left/image_raw", 100, &LightDetect::chatterCallback,this);
  ros::spin();
}
void LightDetect::chatterCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Heard");
  //sensor_msgs::image_encodings ridge_;
  try
  {
    //new frame
    in = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8); // core dumped when using 16 bit
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge_exception: %s",e.what());
    return;
  }
  cv::Mat in1,in0;
  // cv::vector<cv::Mat> IN1;
  cv::vector<cv::Mat> IN0(3),IN1(3),hsv(3);
  cv::Mat hist,backP,drawing;
  int histSize[] = {180,256,256};
  float hrange[] ={0,180}; //range of values
  float srange[] ={0,256}; //range of values
  float vrange[] ={0,256}; //range of values
  int channels[]={0,1,2};
  const float* histRange[] = { hrange, srange,vrange};
  //
  // if (i)
  // {
  //   out = in;
  // }
  cv::split(in->image,IN1);
  // cv::split(out->image,IN0);
  for(i=0;i<3;i++)
  {
  cv::threshold(IN1[i],IN1[i],180,255,0);
  // cv::threshold(IN0[i],IN0[i],180,255,0);
  }
  cv::merge(IN1,in1);
  // cv::merge(IN0,in0);

  cv::cvtColor(in1,in1,CV_BGR2HSV);
  // cv::cvtColor(in0,in0,CV_BGR2HSV);

  // cv::inRange(in->image,cv::Scalar(0,0,0),cv::Scalar(180,255,255),in1);
  // cv::inRange(out->image,cv::Scalar(0,0,0),cv::Scalar(180,255,255),in0);
  // in1 = in1-in0;
  cv::calcHist(&in1,1,channels,cv::Mat(),hist,2,histSize,histRange,true,false);
  cv::split(hist,hsv);
  for(i=0;i<3;i++)
  {
    if (i)
        cv::normalize(hsv[i],hsv[i],0,255,cv::NORM_MINMAX,-1,cv::Mat());
    else
    cv::normalize(hsv[i],hsv[i],0,180,cv::NORM_MINMAX,-1,cv::Mat());
  // cv::threshold(IN0[i],IN0[i],180,255,0);
  }
  cv::calcBackProject(&in1,1,channels,hist,backP,histRange,1,true);
  cv::bitwise_not(backP,backP);
  cv::vector<cv::vector <cv::Point> > contours;
  cv::vector<cv::Vec4i> hi;
  drawing = backP.clone();

  cv::findContours(drawing, contours,hi,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,cv::Point(0,0));
  std::cout<<"contours size: "<<contours.size();
  cv::vector<cv::Point2f> points;
  //out= in;
  for(int j=0,zz=0;j<contours.size();j++)
  {
      cv::drawContours(in->image,contours,j,(180,180,180),1,8,hi,2,cv::Point());
      cv::Moments moment = cv::moments((cv::Mat)contours[j]);
      if (moment.m00)
      {
        points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
        // std::cout<<"m00"<<moment.m00<<"m10"<<moment.m10<<"m01"<<moment.m01;
        std::cout<<"x:"<<points[zz].x<<" y:"<<points[zz].y<<"\n";
        std::cout<<in->image.at<cv::Vec3b>(points[zz].y,points[zz].x)<<"\n";
        zz++;
      }
  }
//
//   cv::vector<cv::Point2f> v;
//   if (contours.size())
//   {
//
//   //gives the center point and the RGB value
//   for ( int zz =0; zz<points.size();zz++)
//   {
//     std::cout<<"\n\nx:"<<points[zz].x<<" y:"<<points[zz].y<<"\n";
//
//     std::cout<<"\n"<<in->image.at<cv::Vec3b>(points[zz].y,points[zz].x)<<"\n";
//   }
// }
    //std::cout<<"Po"<<v;
    //

  //std::vector<int> vec;
  // for (auto vec : contours)
  // {
  //   for (auto v : vec)
  //   {
  //     std::cout<<v<<"\n";
  //   }
  // }
  // for (auto vec : hi)
  //   std::cout << vec << std::endl;

  cv::imshow("out",backP);
  cv::imshow("in",in->image);

  // for the first frame compare the frame with same frame

  //cv::Mat hist,backP;
  // cv::cvtColor(in->image,z.image,CV_BGR2HSV);
  //cv::calcHist(&in1.image,1,{0,1},NULL,hist.image,2,{180,255},{{0,180},{0,255}},true,false);
  //cv::calcBackProject(&in1.image,1,{0,1},hist.image,backP.image,{{0,180},{0,255}},1,true);

  //cv::inRange(z.image,cv::Scalar(0,0,0),cv::Scalar(180,255,255),z.image);
  //cv::threshold(in->image,z.image,120,255,0);
  //in1.image = in->image - z.image;
  //z.header = in->header;
  //cv::imshow("Output",backP.image);
  cv::waitKey(1);
  pub.publish(z.toImageMsg());
  i = 0;
  out = in;

}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "val_lightdetect_node");
  ros::NodeHandle n;
  LightDetect detect(n);
}
