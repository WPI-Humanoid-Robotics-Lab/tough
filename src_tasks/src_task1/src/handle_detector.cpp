#include <src_task1/handle_detector.h>
#include <visualization_msgs/Marker.h>
#include "tough_common/val_common_names.h"
#include <pcl/common/centroid.h>
#include <thread>

#define DISABLE_DRAWINGS true

HandleDetector::HandleDetector(ros::NodeHandle nh) : nh_(nh), ms_sensor_(nh_), organizedCloud_(new tough_perception::StereoPointCloudColor)
{
    ms_sensor_.giveQMatrix(qMatrix_);

    // Initialize the subscribers so that we don't spend time waiting for it to initialize later
    cv::Mat img;
    ms_sensor_.giveImage(img);
    ms_sensor_.giveDisparityImage(img);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",1);
}

void HandleDetector::showImage(cv::Mat image, std::string caption)
{
#ifdef DISABLE_DRAWINGS
    return;
#endif
    cv::namedWindow( caption, cv::WINDOW_AUTOSIZE );
    cv::imshow( caption, image);
    cv::waitKey(0);
}

inline void HandleDetector::colorSegment(const cv::Mat &imgHSV, const int hsv[6], cv::Mat &outImg)
{
    cv::inRange(imgHSV,cv::Scalar(hsv[0],hsv[2],hsv[4]), cv::Scalar(hsv[1],hsv[3],hsv[5]),outImg);
}

void HandleDetector::doMorphology (cv::Mat &image)
{
    cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(15,15)));
    cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10)));
}


void HandleDetector::findMaxContour(const cv::Mat image, cv::Rect &roi)
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
    if (boundRect.empty()){
        return;
    }
#ifndef DISABLE_DRAWINGS
        cv::drawContours( drawing, contours, largest_contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
        cv::rectangle( drawing, boundRect[largest_contour_index].tl(), boundRect[largest_contour_index].br(), color, 2, 8, 0 );
        showImage(drawing);
#endif

    roi = boundRect[largest_contour_index];
}

bool HandleDetector::findAllContours (const cv::Mat &image)
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
        rectCenter_.push_back(currentPoint);
        convexHulls_.push_back(hull[i]);

        std::cout<<rectCenter_.back().x<<" "<<rectCenter_.back().y<<std::endl;
        prev_point = currentPoint;
    }
    showImage(drawing);

    return false;
}

bool HandleDetector::getHandleLocation(std::vector<geometry_msgs::Point>& handleLocs)
{

    bool foundButton = false;
    pcl::PointXYZRGB pclPoint;

//    pcl::removeNaNFromPointCloud(organizedCloud);
    tf::TransformListener listener;

    int threadWaitCounter = 0;

    while(organizedCloud_->empty() && threadWaitCounter++ < 40){
        ros::Duration(0.1).sleep();
    }

    if(organizedCloud_->empty()){
        return false;
    }

    for(int i=0; i<rectCenter_.size(); i++)
    {
        cv::Mat hullPoints = cv::Mat::zeros(current_image_.size(), CV_8UC1);
        cv::fillConvexPoly(hullPoints, convexHulls_[i],cv::Scalar(255));
        showImage(hullPoints);

        cv::Mat nonZeroCoordinates;
        cv::erode(hullPoints, hullPoints, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
        cv::findNonZero(hullPoints, nonZeroCoordinates);
        if (nonZeroCoordinates.total() < 10){
            continue;
        }
        pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud;

        for (int k = 0; k < nonZeroCoordinates.total(); k++ ) {
            pcl::PointXYZRGB temp_pclPoint = organizedCloud_->at(nonZeroCoordinates.at<cv::Point>(k).x, nonZeroCoordinates.at<cv::Point>(k).y);

            if (temp_pclPoint.z > -2.0)
            {
                currentDetectionCloud.push_back(pcl::PointXYZRGB(temp_pclPoint));

            }
        }

        Eigen::Vector4f cloudCentroid;
        //  Calculating the Centroid of the handle Point cloud
        pcl::compute3DCentroid(currentDetectionCloud, cloudCentroid);

        frameID_++;
        buttonCenters_.push_back(pcl::PointXYZ(cloudCentroid(0), cloudCentroid(1), cloudCentroid(2)));

        std::cout<<"Handle "<< i << " at " << buttonCenters_[i].x << " " << buttonCenters_[i].y << " " << buttonCenters_[i].z << std::endl;
        geometry_msgs::PointStamped geom_point;
        geom_point.point.x = cloudCentroid(0);
        geom_point.point.y = cloudCentroid(1);
        geom_point.point.z = cloudCentroid(2);
        geom_point.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;

        try
        {
            listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point, geom_point);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::spinOnce();
            return false;
        }
        if(geom_point.point.z < 0.5f ){
            continue;
        }
        handleLocs.push_back(geom_point.point);
        visualize_point(geom_point.point);
        // return true only if there are 4 points in handleLocs
        foundButton = handleLocs.size() == 4;

    }
    marker_pub_.publish(markers_);
    return foundButton;

}

void HandleDetector::getReducedImage(cv::Mat &image, const cv::Rect &roi)
{
    image = image(roi);
}

bool HandleDetector::findHandles(std::vector<geometry_msgs::Point>& handleLocs)
{
    buttonCenters_.clear();
    rectCenter_.clear();
    convexHulls_.clear();
    handleLocs.clear();
    markers_.markers.clear();

    ms_sensor_.giveImage(current_image_);
    ms_sensor_.giveDisparityImage(current_disparity_);
    organizedCloud_->clear();

    //    Starting a thread with lambda as debugging normal thread call was taking longer. [&] is because we are calling a static function
    std::thread t1([&](){
                tough_perception::PointCloudHelper::generateOrganizedRGBDCloud(current_disparity_, current_image_, qMatrix_, organizedCloud_);
            });
    t1.detach();

    cv::inRange(current_image_, cv::Scalar(73, 0, 0), cv::Scalar(255, 20, 20), imRed_);
    showImage(imRed_);

    cv::inRange(current_image_, cv::Scalar(0, 0, 70), cv::Scalar(20, 20, 255), imBlue_);
    showImage(imBlue_);

    cv::cvtColor(current_image_, current_image_HSV_, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(current_image_HSV_, current_image_HSV_, cv::Size(9, 9), 2, 2);

    side_ = "left"; // used for naming frames. Left = red;

    //    colorSegment(current_image_HSV_, hsvRed_, imRed_);
    doMorphology(imRed_);
    showImage(imRed_);
    findMaxContour(imRed_, roiRed_);
    if(roiRed_.height == 0 || roiRed_.width == 0){
        return false;
    }

    doMorphology(imBlue_);
    showImage(imBlue_);
    findMaxContour(imBlue_, roiBlue_);
    if(roiBlue_.height == 0 || roiBlue_.width == 0){
        return false;
    }

    imRedReduced_= cv::Mat::zeros(current_image_HSV_.size(), current_image_HSV_.type());

    cv::Mat mask = cv::Mat::zeros(current_image_.size(), current_image_.type());
    cv::rectangle(mask, cv::Point(roiRed_.x, roiRed_.y), cv::Point(roiRed_.x+roiRed_.width, roiRed_.y+roiRed_.height),cv::Scalar(255, 255, 255), -1, 8, 0);
    showImage(mask);
    current_image_HSV_.copyTo(imRedReduced_,mask);

    showImage(imRedReduced_);
    colorSegment(imRedReduced_, hsvGray_, imGray_);
    showImage(imGray_);
    showImage(current_disparity_);
    bool val = findAllContours(imGray_);

    side_ = "right"; // used for naming frames;

    //    colorSegment(current_image_HSV_, hsvBlue_, imBlue_);

    imBlueReduced_= cv::Mat::zeros(current_image_HSV_.size(), current_image_HSV_.type());

    cv::Mat maskBlue = cv::Mat::zeros(current_image_.size(), current_image_.type());
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

void HandleDetector::visualize_point(geometry_msgs::Point point){

    std::cout<< "goal origin :\n"<< point << std::endl;
    static int id = 0;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = std::to_string(frameID_);
    marker.id = id++;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    markers_.markers.push_back(marker);
}
