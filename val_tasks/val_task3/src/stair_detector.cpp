#include "val_task3/stair_detector.h"
#include <visualization_msgs/Marker.h>
#include "tough_common/val_common_names.h"
#include <pcl/common/centroid.h>
#include <thread>

#define DISABLE_DRAWINGS true
#define DISABLE_TRACKBAR true

StairDetector::StairDetector(ros::NodeHandle nh) : nh_(nh), ms_sensor_(nh_), organizedCloud_(new src_perception::StereoPointCloudColor), coefficients_(4), endPoints_(2)
{
    ms_sensor_.giveQMatrix(qMatrix_);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",1);
}

void StairDetector::showImage(cv::Mat image, std::string caption)
{
#ifdef DISABLE_DRAWINGS
    return;
#endif
    cv::namedWindow( caption, cv::WINDOW_AUTOSIZE );
    cv::imshow( caption, image);
    cv::waitKey(3);
}

void StairDetector::colorSegment(cv::Mat &imgHSV, cv::Mat &outImg)
{
    //cv::cvtColor(imgHSV, imgHSV, cv::COLOR_BGR2HSV);
    cv::inRange(imgHSV, cv::Scalar(hsv_[0], hsv_[2], hsv_[4]), cv::Scalar(hsv_[1], hsv_[3], hsv_[5]), outImg);
    cv::morphologyEx(outImg, outImg, cv::MORPH_OPEN, getStructuringElement( cv::MORPH_ELLIPSE,cv::Size(3,3)));
    cv::dilate(outImg, outImg, getStructuringElement( cv::MORPH_RECT, cv::Size(3,3)));

#ifdef DISABLE_TRACKBAR
    return;
#endif
    setTrackbar();
    //ROS_INFO_STREAM(outImg);
    cv::imshow("binary thresholding", outImg);
    cv::imshow("Actual Image", current_image_);
    cv::waitKey(3);
}

void StairDetector::findMaxContour(const std::vector<std::vector<cv::Point> >& contours)
{
    int largest_area = 0;
    int second_largest_area = 0;
    int largest_contour_index = 0;
    int second_largest_contour_index = 0;
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
        else if(area > second_largest_area && area < largest_area)
        {
            second_largest_area = area;
            // Store the index of second largest contour
            second_largest_contour_index = i;

        }
    }
    if (largest_area > 500) convexHulls_.push_back(hull[largest_contour_index]);
    if (second_largest_area > 500) convexHulls_.push_back(hull[second_largest_contour_index]);

}

bool StairDetector::getStairLocation(geometry_msgs::Point& stairLocation, uint& numSideBarsDetected )
{
    bool foundStair = false;
    src_perception::StereoPointCloudColor::Ptr organizedCloud(new src_perception::StereoPointCloudColor);
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(current_disparity_, current_image_, qMatrix_, organizedCloud);
    tf::TransformListener listener;
    geometry_msgs::PointStamped geom_point;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imgHSV, outImg;
    Eigen::Vector4f cloudCentroid;
    geometry_msgs::PointStamped dir_vec;
    geom_point.point.x = 0;
    geom_point.point.y = 0;
    geom_point.point.z = 0;
    dir_vec.point.x = 0;
    dir_vec.point.y = 0;
    dir_vec.point.z = 0;

    cv::cvtColor(current_image_, imgHSV, cv::COLOR_BGR2HSV);
    colorSegment(imgHSV, outImg);

    // Find contours
    cv::findContours(outImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


    if(!contours.empty()) //avoid seg fault at runtime by checking that the contours exist
    {
        findMaxContour(contours);

        if (convexHulls_.size() != 2) {
            //ROS_INFO_STREAM("no. of bars " << convexHulls_.size() << std::endl); numSideBarsDetected = convexHulls_.size();
            return false;}

        foundStair = true;
        for(size_t i = 0; i < convexHulls_.size(); i++)
        {
            //ROS_INFO_STREAM("convex hull area" << i << "\t" << cv::contourArea( convexHulls_[i]) << std::endl);
            cv::Mat hullPoints = cv::Mat::zeros(current_image_.size(), CV_8UC1);
            cv::fillConvexPoly(hullPoints, convexHulls_[i],cv::Scalar(255));
            cv::Mat nonZeroCoordinates;
            cv::findNonZero(hullPoints, nonZeroCoordinates);

            if (nonZeroCoordinates.total() < 10)    return false;

            pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud;

            double temp_norm = 20000000.0;
            unsigned int norm_index = 0;
            unsigned int smallest_norm_index = 0;
            for (unsigned int k = 0, index = 0; k < nonZeroCoordinates.total(); k++ )
            {
                pcl::PointXYZRGB temp_pclPoint = organizedCloud->at(nonZeroCoordinates.at<cv::Point>(k).x, nonZeroCoordinates.at<cv::Point>(k).y);
                if (temp_pclPoint.z < -2.0 || temp_pclPoint.y > -0.0)
                {
                    continue;
                }


                double norm = std::pow(temp_pclPoint.x, 2) + std::pow(temp_pclPoint.y, 2) + std::pow(temp_pclPoint.z, 2);
                if (norm < temp_norm)
                {
                    temp_norm = norm;
                    smallest_norm_index = index;
                    norm_index = k;
                }

                currentDetectionCloud.push_back(pcl::PointXYZRGB(temp_pclPoint));
                ++index;
            }


            if(currentDetectionCloud.empty()) {
                numSideBarsDetected = 1;
                // ROS_INFO_STREAM("no. of bars detected due to improper cloud " <<  numSideBarsDetected << std::endl);
                return false;
            }
            numSideBarsDetected = convexHulls_.size();
            cloudCentroid(0) = currentDetectionCloud[smallest_norm_index].x;
            cloudCentroid(1) = currentDetectionCloud[smallest_norm_index].y;
            cloudCentroid(2) = currentDetectionCloud[smallest_norm_index].z;
            endPoints_[i] = cloudCentroid;

            if( foundStair )
            {
                geom_point.point.x += cloudCentroid(0)/2.0;
                geom_point.point.y += cloudCentroid(1)/2.0;
                geom_point.point.z += cloudCentroid(2)/2.0;
                dir_vec.point.x += std::pow(-1, i) * cloudCentroid(0);
                dir_vec.point.y += std::pow(-1, i) * cloudCentroid(1);
                dir_vec.point.z += std::pow(-1, i) * cloudCentroid(2);
                geom_point.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
                dir_vec.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
            }
        }
        try
        {
            listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point, geom_point);
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, dir_vec, dir_vec);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return false;
        }

        stairLocation.x = double (geom_point.point.x);
        stairLocation.y = double (geom_point.point.y);
        stairLocation.z = double (geom_point.point.z);

        geometry_msgs::Point new_point;
        new_point.x = geom_point.point.x + 1;
        double C = (new_point.x - geom_point.point.x)*(dir_vec.point.x);
        new_point.y = geom_point.point.y - C/double(dir_vec.point.y);
        new_point.z = 0;
        geom_point.point.z = 0;
        double geom_norm = std::pow(geom_point.point.x, 2) + std::pow(geom_point.point.y, 2);
        double dir_norm = std::pow(new_point.x, 2) + std::pow(new_point.y, 2);
        if(dir_norm <= geom_norm)
        {
            new_point.x = geom_point.point.x - 1;
            C = (new_point.x - geom_point.point.x)*(dir_vec.point.x);
            new_point.y = geom_point.point.y - C/double(dir_vec.point.y);
        }

        dirVector_.x = new_point.x - geom_point.point.x;
        dirVector_.y = new_point.y - geom_point.point.y;
        dirVector_.z = new_point.z - geom_point.point.z;

        coefficients_[0] = dir_vec.point.x;
        coefficients_[1] = dir_vec.point.y;
        coefficients_[2] = 0;
        coefficients_[3] = (geom_point.point.x * dir_vec.point.x) + (geom_point.point.y * dir_vec.point.y);

        visualize_direction(geom_point.point, new_point);

        marker_pub_.publish(markers_);
    }
    return foundStair;
}



bool StairDetector::findStair(geometry_msgs::Point& stairLoc, uint& numSideBarsDetected)
{
    //VISUALIZATION - include ros::spinOnce();
    convexHulls_.clear();
    //cv::Mat outImg;
    ros::spinOnce();
    markers_.markers.clear();
    if(ms_sensor_.giveImage(current_image_))
    {
        if( ms_sensor_.giveDisparityImage(current_disparity_))
        {
            //colorSegment(current_image_, outImg);
            return getStairLocation(stairLoc, numSideBarsDetected);
            //return true;
        }
    }
    return 0;
}


void StairDetector::setTrackbar()
{
    cv::namedWindow("binary thresholding");
    cv::createTrackbar("hl", "binary thresholding", &hsv_[0], 180);
    cv::createTrackbar("hu", "binary thresholding", &hsv_[1], 180);
    cv::createTrackbar("sl", "binary thresholding", &hsv_[2], 255);
    cv::createTrackbar("su", "binary thresholding", &hsv_[3], 255);
    cv::createTrackbar("vl", "binary thresholding", &hsv_[4], 255);
    cv::createTrackbar("vu", "binary thresholding", &hsv_[5], 255);

    cv::getTrackbarPos("hl", "binary thresholding");
    cv::getTrackbarPos("hu", "binary thresholding");
    cv::getTrackbarPos("sl", "binary thresholding");
    cv::getTrackbarPos("su", "binary thresholding");
    cv::getTrackbarPos("vl", "binary thresholding");
    cv::getTrackbarPos("vu", "binary thresholding");
}


std::vector<double> StairDetector::coefficients() const
{
    return coefficients_;
}

std::vector<Eigen::Vector4f> StairDetector::endPoints() const
{
    return endPoints_;
}

geometry_msgs::Point StairDetector::dirVector() const
{
    return dirVector_;
}

void StairDetector::visualize_point(geometry_msgs::Point point){

//    std::cout<< "goal origin :\n"<< point << std::endl;
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

void StairDetector::visualize_direction(geometry_msgs::Point point1, geometry_msgs::Point point2){

//    std::cout<< "Start origin :\n"<< point1 << std::endl;
//    std::cout<< "End origin :\n"<< point2 << std::endl;
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
    marker.type = visualization_msgs::Marker::ARROW;
    marker.points.push_back(point1);
    marker.points.push_back(point2);
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //marker.pose.position = point;
    //marker.pose.orientation.w = 1.0f;
    marker.scale.x = 0.01;
    marker.scale.y = 0.1;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    markers_.markers.push_back(marker);
}

StairDetector::~StairDetector()
{

}
