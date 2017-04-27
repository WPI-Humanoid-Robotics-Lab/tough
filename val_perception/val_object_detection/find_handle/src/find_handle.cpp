#include <find_handle.h>

cv::Mat img;
cv::Rect roi;
std::vector<cv::Point> rectCenter;

void showImage(cv::Mat image)
{
//    return;
    cv::namedWindow( "Demo", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Demo", image);
    cv::waitKey(0);
}

cv::Mat colorSegment(cv::Mat imgHSV,const int hsv[6])
{
    cv::Mat Image;
    cv::inRange(imgHSV,cv::Scalar(hsv[0],hsv[2],hsv[4]), cv::Scalar(hsv[1],hsv[3],hsv[5]),Image);
    return Image;
}

cv::Mat doMorphology (cv::Mat imgThresholded)
{
//    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
    cv::dilate(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
//    cv::erode(imgThresholded,imgThresholded,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    return imgThresholded;
}

bool findRoi (cv::Mat src_gray, char c)
{
    int thresh = 100;
    cv::RNG rng(12345);
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    bool handle_detected = false;

    // Detect edges using canny
    cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );

    // Find contours
    cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<std::vector<cv::Point> >hull( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
     {
        cv::convexHull( cv::Mat(contours[i]), hull[i], false );
        cv::approxPolyDP( cv::Mat(hull[i]), contours_poly[i], 3, true );
       boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

     }

    int largest_area=0;
    int largest_contour_index=0;
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

    // Draw contours //bounding box
    if (c=='P')
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        for( int i = 0; i< contours.size(); i++ )
         {
           double area = contourArea( contours[i] );  //  Find the area of contour
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
        return handle_detected;
    }

    else if(c=='G')
    {
        int count = 0;
        cv::Point prev_point;
        for( int i = 0; i< contours.size(); i++ )
         {
            std::cout << contours.size() << std::endl;
//            double area = contourArea( contours[i] );
//            if (area < 10.0)
//            {
//                continue;
//            }
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

        if (count >= 4) handle_detected = true;
        return handle_detected ;
    }
}

void getHandleLocation(ros::NodeHandle &nh)
{
    src_perception::MultisenseImage mi(nh); // subscribe to image_rect
    cv::Mat gimg, imgHSV, imgSegRectangle, grayArea;
    cv::Mat reducedImageHSV;
    bool flag = false;

    const int hsvGray[6] = {0, 255, 0, 20, 0, 140}; // lh, hh, ls, hs, lv, hv
    const int hsvOrange[6] = {11, 25, 165, 255, 140, 255};
    const int hsvRed[6] = {0, 10, 50, 255, 60, 255};
    const int hsvBlue[6] = {100, 125, 100, 255, 0, 255};

    flag = mi.giveImage(img);

//  Change to grayscale,HSV and add blur
    cv::cvtColor( img, gimg, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur( gimg, gimg, cv::Size(9, 9), 2, 2 );
    cv::cvtColor( img, imgHSV, cv::COLOR_BGR2HSV);
    cv::GaussianBlur( imgHSV, imgHSV, cv::Size(9, 9), 2, 2 );

    imgSegRectangle = colorSegment(imgHSV, hsvOrange);
    imgSegRectangle = doMorphology(imgSegRectangle);
    showImage(imgSegRectangle);
    flag = findRoi(imgSegRectangle, 'G'); // stores result in global variable roi
    reducedImageHSV = imgHSV(roi);
    grayArea = colorSegment(reducedImageHSV, hsvGray);
    showImage(grayArea);
    findRoi(grayArea, 'G');

//    reducedImageHSV = imgHSV(roi);
//    grayArea = colorSegment(reducedImageHSV, hsvGray);
//    flag = findRoi(grayArea, 'G');
//    showImage(grayArea);

}

int main(int argc, char** argv) {
    ros::init (argc,argv,"findHandle");
    ros::NodeHandle nh;
    bool handle_detected = false;

    getHandleLocation(nh);

    return 0;
}
