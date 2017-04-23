#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <perception_common/MultisenseImage.h>
#include <iostream>



void getHandleLocation(ros::NodeHandle &nh){
    src_perception::MultisenseImage mi(nh);
    cv::Mat img, gimg, imgHSV, imgThresholded;
    bool flag = false;
    int lh=0,ls=0, lv=0;
    int hh=255, hs=4, hv=55;

    flag = mi.giveImage(img);

//  Change to grayscale and add blur
    cv::cvtColor( img, gimg, cv::COLOR_BGR2GRAY );
//    cv::GaussianBlur( gimg, gimg, cv::Size(9, 9), 2, 2 );

//  HSV color segmentation
    cv::cvtColor( img, imgHSV, cv::COLOR_BGR2HSV);
    cv::inRange(imgHSV,cv::Scalar(lh,ls,lv), cv::Scalar(hh,hs,hv),imgThresholded);

    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
//    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
//    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));

    cv::namedWindow( "Hough Circle Transform Demo", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Hough Circle Transform Demo", imgThresholded);
    cv::waitKey(0);

//  Hough
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles( imgThresholded, circles, CV_HOUGH_GRADIENT, 1, img.rows/14, 30, 25, 0, 0 );

    for( size_t i = 0; i < circles.size(); i++ ) {
       cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
       int radius = cvRound(circles[i][2]);
       // circle center
       circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
       // circle outline
       circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
     }

    cv::namedWindow( "Hough Circle Transform Demo", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Hough Circle Transform Demo", img );
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    ros::init (argc,argv,"findHandle");
    ros::NodeHandle nh;
    getHandleLocation(nh);
    return 0;
}
