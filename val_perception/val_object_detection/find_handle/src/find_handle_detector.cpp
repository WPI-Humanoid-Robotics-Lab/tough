#include <find_handle.h>

handle_detector::handle_detector(ros::NodeHandle &nh)
{
    src_perception::MultisenseImage mi(nh); // subscribe to image_rect
    mi.giveImage(mi_);
    cv::cvtColor(mi_, imgHSV_, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(imgHSV_, imgHSV_, cv::Size(9, 9), 2, 2);
}

void handle_detector::showImage(cv::Mat image)
{
    cv::namedWindow( "Handle Detection", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Handle Detection", image);
    cv::waitKey(0);
}

cv::Mat handle_detector::colorSegment(cv::Mat imgHSV, const int hsv[6])
{
    cv::Mat Image;
    cv::inRange(imgHSV,cv::Scalar(hsv[0],hsv[2],hsv[4]), cv::Scalar(hsv[1],hsv[3],hsv[5]),Image);
    return Image;
}

cv::Mat handle_detector::doMorphology (cv::Mat image)
{
    cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(15,15)));
    cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
    return image;
}


cv::Rect handle_detector::findMaxContour (cv::Mat image)
{
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
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
    return roi;
}

bool handle_detector::findAllContours (cv::Mat image)
{
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
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

void handle_detector::getHandleLocation()
{
    return;
}

cv::Mat handle_detector::getReducedImage(cv::Mat image, cv::Rect roi)
{
    image = image(roi);
    return image;
}

void handle_detector::findHandles()
{
    imRed_ = colorSegment(imgHSV_, hsvRed_);
    imRed_ = doMorphology(imRed_);
    showImage(imRed_);
    findAllContours(imRed_);
    roiRed_ = findMaxContour(imRed_);
    imRedReduced_ = getReducedImage(imRed_, roiRed_);
    imGray_ = colorSegment(imRedReduced_, hsvGray_);
    showImage(imGray_);
}

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findHandleDetector");
    ros::NodeHandle nh;
    handle_detector h1(nh);
    h1.findHandles();
}
