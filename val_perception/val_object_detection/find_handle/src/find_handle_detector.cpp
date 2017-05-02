#include <find_handle.h>
#include <visualization_msgs/Marker.h>

void handle_detector::showImage(cv::Mat image)
{
    return;
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

bool handle_detector::getHandleLocation()
{

    bool foundButton = false;
    pcl::PointXYZRGB pclPoint;
    src_perception::StereoPointCloudColor::Ptr organizedCloud(new src_perception::StereoPointCloudColor);
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(disp_, mi_, qMatrix_, organizedCloud);
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster br;


    try
    {
        listener.waitForTransform("/world", "/left_camera_optical_frame", ros::Time(0), ros::Duration(3.0));
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
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_camera_optical_frame", ss.str()));

        frameID_++;
        buttonCenters_.push_back(pclPoint);

        std::cout<<"Handle "<< i << " at " << buttonCenters_[i].x << " " << buttonCenters_[i].y << " " << buttonCenters_[i].z << std::endl;
        foundButton = true;
    }



    return foundButton;

}

cv::Mat handle_detector::getReducedImage(cv::Mat image, cv::Rect roi)
{
    image = image(roi);
    return image;
}

bool handle_detector::findHandles(ros::NodeHandle &nh, std::vector<geometry_msgs::Point>& handleLocs)
{
    src_perception::MultisenseImage mi(nh); // subscribe to image_rect
    mi.giveImage(mi_);
    cv::cvtColor(mi_, imgHSV_, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(imgHSV_, imgHSV_, cv::Size(9, 9), 2, 2);

    mi.giveDisparityImage(disp_);
    mi.giveQMatrix(qMatrix_);

    side_ = "left"; // used for naming frames. Left = red;

    imRed_ = colorSegment(imgHSV_, hsvRed_);
    imRed_ = doMorphology(imRed_);
    showImage(imRed_);
    roiRed_ = findMaxContour(imRed_);
    imRedReduced_= cv::Mat::zeros(imgHSV_.size(), imgHSV_.type());

    cv::Mat mask = cv::Mat::zeros(imgHSV_.size(), imgHSV_.type());
    cv::rectangle(mask, cv::Point(roiRed_.x, roiRed_.y), cv::Point(roiRed_.x+roiRed_.width, roiRed_.y+roiRed_.height),cv::Scalar(255, 255, 255), -1, 8, 0);
    showImage(mask);
    imgHSV_.copyTo(imRedReduced_,mask);

    showImage(imRedReduced_);
    imGray_ = colorSegment(imRedReduced_, hsvGray_);
    showImage(imGray_);
    showImage(disp_);
    bool val=false;
    val = findAllContours(imGray_);


    side_ = "right"; // used for naming frames;

    imBlue_ = colorSegment(imgHSV_, hsvBlue_);
    imBlue_ = doMorphology(imBlue_);
    showImage(imBlue_);
    roiBlue_ = findMaxContour(imBlue_);
    imBlueReduced_= cv::Mat::zeros(imgHSV_.size(), imgHSV_.type());

    cv::Mat maskBlue = cv::Mat::zeros(imgHSV_.size(), imgHSV_.type());
    cv::rectangle(maskBlue, cv::Point(roiBlue_.x, roiBlue_.y), cv::Point(roiBlue_.x+roiBlue_.width, roiBlue_.y+roiBlue_.height),cv::Scalar(255, 255, 255), -1, 8, 0);
    showImage(maskBlue);
    imgHSV_.copyTo(imBlueReduced_,maskBlue);

    showImage(imBlueReduced_);
    imGray_ = colorSegment(imBlueReduced_, hsvGray_);
    showImage(imGray_);
    showImage(disp_);
    val = findAllContours(imGray_);
    getHandleLocation();

    return false;
}

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findHandleDetector");
    ros::NodeHandle nh;
    int numIterations = 0;
    bool foundButton = false;
    std::vector<geometry_msgs::Point> handleLocs;
    handle_detector h1;

    while (!foundButton && numIterations <1)
    {
        foundButton = h1.findHandles(nh, handleLocs);
        numIterations++;
    }

}
//     Obtaining a stereo point cloud for Z position and RGB values

//    for(int i=0; i<contours.size(); i++)
//    {
//        moment[i] = cv::moments((cv::Mat)contours[i]);
//        if (moment[i].m00)
//        {
//            point = cv::Point2f(moment[i].m10/moment[i].m00,moment[i].m01/moment[i].m00);

//            ROS_DEBUG("m00:%.2f, m10:%.2f, m01:%.2f",moment[i].m00, moment[i].m10, moment[i].m01);
//            ROS_DEBUG("x:%d, y:%d", int(point.x), int(point.y));


//            //assign the button center
//            tempButtonCenter.x = int (point.x);
//            tempButtonCenter.y = int (point.y);

//            pclPoint = organizedCloud->at(tempButtonCenter.x, tempButtonCenter.y);

//            transform.setOrigin( tf::Vector3(pclPoint.x , pclPoint.y, pclPoint.z));
//            q.setRPY(0, 0, 0);
//            transform.setRotation(q);
//            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_camera_optical_frame", frame));

//            buttonCenters_.push_back(pclPoint);
//            visualize_point(buttonCenters_[i]);

//            std::cout<<"Handle "<< i << " at " << buttonCenters_[i].x << " " << buttonCenters_[i].y << std::endl;
//            foundButton = true;
//        }
//    }
