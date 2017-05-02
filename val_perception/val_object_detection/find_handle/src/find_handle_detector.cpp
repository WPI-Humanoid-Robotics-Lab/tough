#include <find_handle.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

void visualize_point(pcl::PointXYZRGB goal){

//    std::cout<< "goal origin :\n"<< goal.position << std::endl;
//    std::cout<< "goal orientation :\n"<< goal.orientation << std::endl;


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "test_object_detector";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = goal.x;
    marker.pose.position.y = goal.y;
    marker.pose.position.z = goal.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 3;
    marker.scale.y = 3;
    marker.scale.z = 3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_pub.publish(marker);
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

    if(!contours.empty()) //avoid seg fault at runtime by checking that the contours exist
    {
        getHandleLocation(contours);
        return true;
    }

    return false;
}

bool handle_detector::getHandleLocation(std::vector<std::vector<cv::Point>> contours)
{

    bool foundButton = false;
    cv::Point2f point;
    std::vector<cv::Moments> moment(contours.size());
    geometry_msgs::Point tempButtonCenter;
    pcl::PointXYZRGB pclPoint;
    src_perception::StereoPointCloudColor::Ptr organizedCloud(new src_perception::StereoPointCloudColor);
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(disp_, mi_, qMatrix_, organizedCloud);
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Quaternion q;
    static tf::TransformBroadcaster br;

    std::string frame = "handleFrame";

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


//     Obtaining a stereo point cloud for Z position and RGB values

    for(int i=0; i<contours.size(); i++)
    {
        moment[i] = cv::moments((cv::Mat)contours[i]);
        if (moment[i].m00)
        {
            point = cv::Point2f(moment[i].m10/moment[i].m00,moment[i].m01/moment[i].m00);

            ROS_DEBUG("m00:%.2f, m10:%.2f, m01:%.2f",moment[i].m00, moment[i].m10, moment[i].m01);
            ROS_DEBUG("x:%d, y:%d", int(point.x), int(point.y));


            //assign the button center
            tempButtonCenter.x = int (point.x);
            tempButtonCenter.y = int (point.y);

            pclPoint = organizedCloud->at(tempButtonCenter.x, tempButtonCenter.y);

            transform.setOrigin( tf::Vector3(pclPoint.x , pclPoint.y, pclPoint.z));
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_camera_optical_frame", frame));

            buttonCenters_.push_back(pclPoint);
            visualize_point(buttonCenters_[i]);
    
            std::cout<<"Handle "<< i << " at " << buttonCenters_[i].x << " " << buttonCenters_[i].y << std::endl;
            foundButton = true;
        }
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

    imRed_ = colorSegment(imgHSV_, hsvRed_);
    imRed_ = doMorphology(imRed_);
    showImage(imRed_);
    roiRed_ = findMaxContour(imRed_);
    imRedReduced_= cv::Mat::zeros(imgHSV_.size(), imgHSV_.type());

    cv::Mat mask = cv::Mat::zeros(imgHSV_.size(), imgHSV_.type());
    cv::rectangle(imRedReduced_, cv::Point(roiRed_.x, roiRed_.y), cv::Point(roiRed_.x+roiRed_.width, roiRed_.y+roiRed_.height),cv::Scalar(255, 255, 255), -1, 8, 0);

//    imgHSV_.copyTo(imRedReduced_,mask);
    cv::bitwise_or(mask, imgHSV_, imRedReduced_);
    showImage(imRedReduced_);
//    imGray_ = colorSegment(imRedReduced_, hsvGray_);
//    showImage(imGray_);

    bool val=false;
//    val = findAllContours(imGray_);
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

    marker_pub = nh.advertise<visualization_msgs::Marker>("/field/visualization_marker", 1);
    while (!foundButton && numIterations <1)
    {
        foundButton = h1.findHandles(nh, handleLocs);
        numIterations++;
    }

}


//    imGray_ = colorSegment(imgHSV_, hsvGray_);
//    imOrange_ = colorSegment(imgHSV_, hsvOrange_);
//    cv::erode(imOrange_, imOrange_, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(15,15)));
//    roiOrange_ = findMaxContour(imOrange_);
//    imOrange_ = cv::Mat::zeros(imOrange_.size(),CV_8U);
//    imOrange_(roiOrange_) = cv::Mat::ones(roiOrange_.height, roiOrange_.width,CV_8U);
//    cv::bitwise_and(imGray_, imOrange_, imGray_);
//    showImage(imGray_);
//    showImage(imOrange_);
