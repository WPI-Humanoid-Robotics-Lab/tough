#include <val_task2/button_detector.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/centroid.h>
#include "val_common/val_common_names.h"

#define DISABLE_DRAWINGS true

button_detector::button_detector(ros::NodeHandle nh) : nh_(nh), ms_sensor_(nh_)
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_buttons",1);
    ms_sensor_.giveQMatrix(qMatrix_);
}

void button_detector::showImage(cv::Mat image, std::string caption)
{
#ifdef DISABLE_DRAWINGS
    return;
#endif
    cv::namedWindow( caption, cv::WINDOW_AUTOSIZE );
    cv::imshow( caption, image);
    cv::waitKey(0);
}

inline void button_detector::colorSegment(const cv::Mat &imgHSV, const int hsvl[6], const int hsvu[6], cv::Mat &outImg)
{
    cv::Mat mask1, mask2;
    cv::inRange(imgHSV,cv::Scalar(hsvl[0],hsvl[2],hsvl[4]), cv::Scalar(hsvl[1],hsvl[3],hsvl[5]),mask1);
    cv::inRange(imgHSV,cv::Scalar(hsvu[0],hsvu[2],hsvu[4]), cv::Scalar(hsvu[1],hsvu[3],hsvu[5]),mask2);
    outImg = mask1 | mask2;
}

size_t button_detector::findMaxContour(const std::vector<std::vector<cv::Point> >& contours)
{
    int largest_area = 0;
    int largest_contour_index = 0;
    std::vector<std::vector<cv::Point>> hull(contours.size());

    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::convexHull(contours[i], hull[i], false);
        //  Find the area of contour
        double area = cv::contourArea( hull[i]);

        if(area > largest_area)
        {
            largest_area = area;
            // Store the index of largest contour
            largest_contour_index = i;
        }
    }

    convexHulls_ = hull[largest_contour_index];

    return largest_contour_index;
}

bool button_detector::getButtonLocation(geometry_msgs::Point& buttonLoc)
{

    bool foundButton = false;
    src_perception::StereoPointCloudColor::Ptr organizedCloud(new src_perception::StereoPointCloudColor);
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(current_disparity_, current_image_, qMatrix_, organizedCloud);
    tf::TransformListener listener;
    geometry_msgs::PointStamped geom_point;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imgHSV, outImg;

    cv::cvtColor(current_image_, imgHSV, cv::COLOR_BGR2HSV);
    colorSegment(imgHSV, hsvLowRed_, hsvHighRed_, outImg);

    // Find contours
    cv::findContours(outImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


    if(!contours.empty()) //avoid seg fault at runtime by checking that the contours exist
    {
        cv::Moments moment = cv::moments((cv::Mat)contours[findMaxContour(contours)]);

        if (moment.m00)
        {
            cv::Point2f point = cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00);

            ROS_DEBUG("m00:%.2f, m10:%.2f, m01:%.2f",moment.m00, moment.m10, moment.m01);
            ROS_DEBUG("x:%d, y:%d", int(point.x), int(point.y));

            buttonLoc.x = int (point.x);
            buttonLoc.y = int (point.y);
            foundButton = true;
        }
    }

    cv::Mat hullPoints = cv::Mat::zeros(current_image_.size(), CV_8UC1);
    cv::fillConvexPoly(hullPoints, convexHulls_,cv::Scalar(255));
    cv::Mat nonZeroCoordinates;
    cv::erode(hullPoints, hullPoints, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    cv::findNonZero(hullPoints, nonZeroCoordinates);
    if (nonZeroCoordinates.total() < 10){
        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud;

    for (int k = 0; k < nonZeroCoordinates.total(); k++ ) {
        pcl::PointXYZRGB temp_pclPoint = organizedCloud->at(nonZeroCoordinates.at<cv::Point>(k).x, nonZeroCoordinates.at<cv::Point>(k).y);

        if (temp_pclPoint.z > -2.0)
        {
            currentDetectionCloud.push_back(pcl::PointXYZRGB(temp_pclPoint));

        }
    }
    Eigen::Vector4f cloudCentroid;
    //  Calculating the Centroid of the handle Point cloud
    pcl::compute3DCentroid(currentDetectionCloud, cloudCentroid);
    if( foundButton )
    {
        geom_point.point.x = cloudCentroid(0);
        geom_point.point.y = cloudCentroid(1);
        geom_point.point.z = cloudCentroid(2);
        geom_point.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
        try
        {
            listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point, geom_point);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }
    }
    //buttonLoc.x = double (geom_point.point.x);
    //buttonLoc.y = double (geom_point.point.y);
    //buttonLoc.z = double (geom_point.point.z);

    visualize_point(geom_point.point);

    marker_pub_.publish(markers_);
    return foundButton;

}

bool button_detector::findButtons(geometry_msgs::Point &buttonLoc)
{
    markers_.markers.clear();

    if(ms_sensor_.giveImage(current_image_))
    {
        if( ms_sensor_.giveDisparityImage(current_disparity_))
        {
            return getButtonLocation(buttonLoc);
        }
    }
    return 0;
}

void button_detector::visualize_point(const geometry_msgs::Point &point){

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
