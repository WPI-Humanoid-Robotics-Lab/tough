#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <perception_common/MultisenseImage.h>
#include <iostream>
#include <vector>
cv::Mat img;
cv::Rect roi;
std::vector<cv::Point> rectCenter(4);

cv::Mat findCircles(cv::Mat imgThresh) {

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles( imgThresh, circles, CV_HOUGH_GRADIENT, 1, img.rows/14, 70, 25, 0, 0 );

    for( size_t i = 0; i < circles.size(); i++ ) {
       cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
       int radius = cvRound(circles[i][2]);
       // circle center
       circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
       // circle outline
       circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
     }
    return imgThresh;
}

void showImage(cv::Mat image) {

    cv::namedWindow( "Demo", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Demo", image);
    cv::waitKey(0);
}

cv::Mat colorSegment(cv::Mat imgHSV, int hsv[6]){

    cv::Mat Image;
    cv::inRange(imgHSV,cv::Scalar(hsv[0],hsv[2],hsv[4]), cv::Scalar(hsv[1],hsv[3],hsv[5]),Image);
    return Image;
}

void doMorphology (cv::Mat imgThresholded) {

//    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
//    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));

}

void findRoi (cv::Mat src_gray, char c) {
    int thresh = 100;
    cv::RNG rng(12345);
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // Detect edges using canny
    cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );
    // Find contours
    cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
     { cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
       //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
     }

    int largest_area=0;
    int largest_contour_index=0;
    cv::Rect bounding_rect;
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

    // Draw contours //bounding box
    if (c=='P') {
        for( int i = 0; i< contours.size(); i++ )
         {
           double area = contourArea( contours[i] );  //  Find the area of contour
           if( area > largest_area ){
               largest_area = area;
               largest_contour_index = i;               //Store the index of largest contour
           }
         }
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours, largest_contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
        cv::rectangle( drawing, boundRect[largest_contour_index].tl(), boundRect[largest_contour_index].br(), color, 2, 8, 0 );
        showImage(drawing);
        roi = boundRect[largest_contour_index];
    }

    else if(c=='G'){
        int count = 0;
        for( int i = 0; i< contours.size(); i++ )
         {
            double area = contourArea( contours[i] );
            if (area < 10.0) i++;
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
            cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
            rectCenter[count] = cv::Point((boundRect[i].x + boundRect[i].width)*0.5, (boundRect[i].y + boundRect[i].height)*0.5);
            std::cout<<rectCenter[count].x<<" "<<rectCenter[count].y<<std::endl;
            count++;
         }
        showImage(drawing);
    }
}

void getHandleLocation(ros::NodeHandle &nh){

    src_perception::MultisenseImage mi(nh); // subscribe to image_rect
    cv::Mat gimg, imgHSV, imgThresholded, imgSegRectangle,nonZeroLoc;
    cv::Mat reducedImageHSV;
    bool flag = false;
    int hsvGray[6] = {0, 255, 0, 10, 0, 255}; // lh, hh, ls, hs, lv, hv
    int hsvOrange[6] = {11, 25, 165, 255, 140, 255};
    int hsvRed[6] = {0, 10, 50, 255, 60, 255};
    int hsvBlue[6] = {100, 125, 100, 255, 50, 255};

    flag = mi.giveImage(img);

//  Change to grayscale,HSV and add blur
    cv::cvtColor( img, gimg, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur( gimg, gimg, cv::Size(9, 9), 2, 2 );
    cv::cvtColor( img, imgHSV, cv::COLOR_BGR2HSV);
    cv::GaussianBlur( imgHSV, imgHSV, cv::Size(3, 3), 2, 2 );

//    std::cout << "orange" << std::endl;
//  HSV color segmentation
//    imgSegRectangle = colorSegment(imgHSV, hsvOrange);
//    doMorphology(imgSegRectangle);
//    showImage(imgSegRectangle);
//    findRoi(imgSegRectangle,'P'); // argument P for panel
//    reducedImageHSV = imgHSV(roi);

    std::cout << "Red" << std::endl;

    imgSegRectangle = colorSegment(imgHSV, hsvRed);
    doMorphology(imgSegRectangle);
    showImage(imgSegRectangle);
    std::cout << "find blue roi start" << std::endl;
    findRoi(imgSegRectangle,'P'); // argument P for panel
    reducedImageHSV = imgHSV(roi);
    std::cout << "find blue roi end" << std::endl;

    std::cout << "gray seg" << std::endl;
    imgThresholded = colorSegment(reducedImageHSV, hsvGray); // gray
    showImage(imgThresholded);
//    findCircles(imgThresholded);
//    showImage(img);
    findRoi(imgThresholded,'G'); // bounding boxes for gray parts
}

int main(int argc, char** argv) {
    ros::init (argc,argv,"findHandle");
    ros::NodeHandle nh;
    getHandleLocation(nh);
    return 0;
}



//    imgSegRectangle = colorSegment(imgHSV, imgThresholded, hsvOrange);
//    cv::findNonZero(imgSegRectangle, nonZeroLoc);
//    cv::bitwise_and(imgThresholded, imgSegRectangle, nonZeroLoc);
//    cv::Rect r(nonZeroLoc);
//    cv::Mat roiimg = img(nonZeroLoc);
//int y = NonZero_Locations.at<Point>(NonZero_Locations.total()-1).y;
