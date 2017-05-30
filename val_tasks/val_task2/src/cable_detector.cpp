#include "val_task2/cable_detector.h"
#include <visualization_msgs/Marker.h>
#include "val_common/val_common_names.h"
#include <pcl/common/centroid.h>
#include <thread>

#define DISABLE_DRAWINGS true
#define DISABLE_TRACKBAR true

CableDetector::CableDetector(ros::NodeHandle nh) : nh_(nh), ms_sensor_(nh_), organizedCloud_(new src_perception::StereoPointCloudColor), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    ms_sensor_.giveQMatrix(qMatrix_);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_cable",1);
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &CableDetector::cloudCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_table/cloud2", 1, true);
    eigenVecs_.reserve(2);
}

void CableDetector::cloudCB(const sensor_msgs::PointCloud2::Ptr& input)
{
    if (input->data.empty())
        return;
    //markers_.markers.clear();
    //add mutex
    mtx_.lock();
    pcl::fromROSMsg(*input, *cloud_);
    mtx_.unlock();
    //ROS_INFO_STREAM("Input Cloud Size : " << cloud_->size() << std::endl);
}

bool CableDetector::getStandPosition(geometry_msgs::Point& stand_loc)
{
    bool stand_detected = false;

    if (cloud_->empty())
        return false;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    mtx_.lock();
    input_cloud = cloud_;
    mtx_.unlock();

    if (input_cloud->empty())
        return false;

    stand_detected = true;
    planeSegmentation(input_cloud);

    return stand_detected;
}

void CableDetector::showImage(cv::Mat image, std::string caption)
{
#ifdef DISABLE_DRAWINGS
    return;
#endif
    cv::namedWindow( caption, cv::WINDOW_AUTOSIZE );
    cv::imshow( caption, image);
    cv::waitKey(3);
}

void CableDetector::colorSegment(cv::Mat &imgHSV, cv::Mat &outImg)
{
    cv::inRange(imgHSV,cv::Scalar(hsv_[0], hsv_[2], hsv_[4]), cv::Scalar(hsv_[1], hsv_[3], hsv_[5]), outImg);
#ifdef DISABLE_TRACKBAR
    return;
#endif
    setTrackbar();

    cv::imshow("binary thresholding", outImg);
    cv::imshow("HSV thresholding", imgHSV);
    cv::waitKey(3);
}

size_t CableDetector::findMaxContour(const std::vector<std::vector<cv::Point> >& contours)
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

    return largest_contour_index;
}

bool CableDetector::getCableLocation(geometry_msgs::Point& cableLoc)
{
    bool foundCable = false;
    src_perception::StereoPointCloudColor::Ptr organizedCloud(new src_perception::StereoPointCloudColor);
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(current_disparity_, current_image_, qMatrix_, organizedCloud);
    tf::TransformListener listener;
    geometry_msgs::PointStamped geom_point;
    geometry_msgs::PointStamped geom_point1;
    geometry_msgs::PointStamped geom_point2;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imgHSV, outImg;
    std::vector<cv::Point> points;

    cv::cvtColor(current_image_, imgHSV, cv::COLOR_BGR2HSV);
    colorSegment(imgHSV, outImg);

    // Find contours
    cv::findContours(outImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    Eigen::Vector4f cloudCentroid0;
    Eigen::Vector4f cloudCentroid1;
    Eigen::Vector4f cloudCentroid2;
    if(!contours.empty()) //avoid seg fault at runtime by checking that the contours exist
    {
        foundCable = true;
        points = getOrientation(contours[findMaxContour(contours)], outImg);
        //ROS_INFO_STREAM(point << std::endl);
        double theta = (1/3600.0) * 3.14159265359;
        double r = std::sqrt(std::pow(points[1].x - points[0].x , 2) + std::pow(points[1].y - points[0].y , 2)) + 5.0;
        pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud0;
        pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud1;
        pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud2;

        for (size_t k = 0; k < 10; k++ )
        {
            for (size_t l = 0; l < 10; l++ )
            {
                pcl::PointXYZRGB temp_pclPoint0;
                pcl::PointXYZRGB temp_pclPoint1;
                pcl::PointXYZRGB temp_pclPoint2;
                try
                {
                    //temp_pclPoint = organizedCloud->at(int(points[1].x + r * std::cos(k * l * theta)), int(points[1].y + r * std::sin(k * l * theta)));
                    temp_pclPoint0 = organizedCloud->at(points[0].x + k, points[0].y + l );
                    temp_pclPoint1 = organizedCloud->at(points[1].x + k, points[1].y + l);
                    temp_pclPoint2 = organizedCloud->at(points[2].x + k, points[2].y + l);
                }
                catch (const std::out_of_range& ex){
                    ROS_ERROR("%s",ex.what());
                    return false;
                }

                if (temp_pclPoint0.z > -2.0 && temp_pclPoint0.z < 2.0 )
                {
                    currentDetectionCloud0.push_back(pcl::PointXYZRGB(temp_pclPoint0));
                }

                if (temp_pclPoint1.z > -2.0 && temp_pclPoint1.z < 2.0 )
                {
                    currentDetectionCloud1.push_back(pcl::PointXYZRGB(temp_pclPoint1));
                }

                if (temp_pclPoint2.z > -2.0 && temp_pclPoint2.z < 2.0 )
                {
                    currentDetectionCloud2.push_back(pcl::PointXYZRGB(temp_pclPoint2));
                }
            }
        }
        //  Calculating the Centroid of the handle Point cloud
        pcl::compute3DCentroid(currentDetectionCloud0, cloudCentroid0);
        pcl::compute3DCentroid(currentDetectionCloud1, cloudCentroid1);
        pcl::compute3DCentroid(currentDetectionCloud2, cloudCentroid2);
    }

    if( foundCable )
    {
        //pcl::PointXYZRGB temp_pclPoint2 = organizedCloud->at(points[1].x, points[1].y);
        geom_point.point.x = cloudCentroid0(0);
        geom_point.point.y = cloudCentroid0(1);
        geom_point.point.z = cloudCentroid0(2);
        geom_point.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;

        geom_point1.point.x = cloudCentroid1(0);
        geom_point1.point.y = cloudCentroid1(1);
        geom_point1.point.z = cloudCentroid1(2);
        geom_point1.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;

        geom_point2.point.x = cloudCentroid2(0);
        geom_point2.point.y = cloudCentroid2(1);
        geom_point2.point.z = cloudCentroid2(2);
        geom_point2.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
        try
        {
            listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point, geom_point);
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point1, geom_point1);
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point2, geom_point2);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }
    }
    cableLoc.x = double (geom_point.point.x);
    cableLoc.y = double (geom_point.point.y);
    cableLoc.z = double (geom_point.point.z);

    cableLoc_.x = double (geom_point2.point.x);
    cableLoc_.y = double (geom_point2.point.y);
    cableLoc_.z = double (geom_point2.point.z);

    //visualize_point(geom_point.point, 0.7, 0.5, 0.0);
    //visualize_point(geom_point1.point, 0.0, 0.0, 1.0);
    //visualize_point(geom_point2.point, 1.0, 0.8, 0.3);
    //visualize_direction(geom_point2.point, geom_point1.point);

    //marker_pub_.publish(markers_);

    return foundCable;
}

std::vector<cv::Point> CableDetector::getOrientation(const std::vector<cv::Point> &contourPts, cv::Mat &img)
{
    int sz = static_cast<int>(contourPts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);

    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = contourPts[i].x;
        data_pts.at<double>(i, 1) = contourPts[i].y;
    }

    //Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

    //Store the center of the object
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
    static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

    //Store the eigenvalues and eigenvectors
    //std::vector<cv::Point2d> eigenVecs_(2);
    std::vector<double> eigen_val(2);

    for (int i = 0; i < 2; ++i)
    {
        eigenVecs_[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
        pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    cv::circle(current_image_, cntr, 3, cv::Scalar(255, 0, 255), 2);
    cv::Point p1 = cntr + 0.03 * cv::Point(static_cast<int>(eigenVecs_[0].x * eigen_val[0]), static_cast<int>(eigenVecs_[0].y * eigen_val[0]));
    cv::Point p2 = cntr - 0.03 * cv::Point(static_cast<int>(eigenVecs_[0].x * eigen_val[0]), static_cast<int>(eigenVecs_[0].y * eigen_val[0]));
    //cv::Point p2 = cntr + 1.0 * cv::Point(static_cast<int>(eigenVecs_[1].x * eigen_val[1]), static_cast<int>(eigenVecs_[1].y * eigen_val[1]));

    //VISUALIZATION - Uncomment this line
    drawAxis(current_image_, cntr, p1, cv::Scalar(0, 255, 0), 1);
    drawAxis(current_image_, cntr, p2, cv::Scalar(255, 255, 0), 1);
    //ROS_INFO_STREAM(p2 << std::endl);
    std::vector<cv::Point> points(3);
    points[0] = cntr;
    points[1] = p1;
    points[2] = p2;
    showImage(current_image_);
    return points;
}

bool CableDetector::findCable(geometry_msgs::Point& cableLoc)
{
    //VISUALIZATION - include a spinOnce here to visualize the eigenvectors
    markers_.markers.clear();
    if(ms_sensor_.giveImage(current_image_))
    {
        if( ms_sensor_.giveDisparityImage(current_disparity_))
        {
            return getCableLocation(cableLoc);
        }
    }
    return 0;
}

void CableDetector::drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale)
{
    double angle;
    double hypotenuse;

    angle = std::atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = std::sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    // double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
    // cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale

    q.x = (int) (p.x - scale * hypotenuse * std::cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * std::sin(angle));
    cv::line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * std::cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * std::sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * std::cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * std::sin(angle - CV_PI / 4));
    cv::line(img, p, q, colour, 1, CV_AA);
}

void CableDetector::setTrackbar()
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

void CableDetector::visualize_point(geometry_msgs::Point point, double r, double g, double b){

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
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    markers_.markers.push_back(marker);
}

void CableDetector::visualize_direction(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
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
    marker.scale.y = 0.05;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    markers_.markers.push_back(marker);
}

bool CableDetector::planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    tf::TransformListener listener;
    markers_.markers.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < input->size(); i++)
    {
        if (std::abs(input->points[i].z - cableLoc_.z) < 0.03)
            output_cloud->points.push_back(input->points[i]);
    }

    Eigen::Vector4f centroid;
    geometry_msgs::PointStamped geom_point;
    geometry_msgs::PointStamped geom_point1;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(output_cloud);
    pass.setFilterFieldName ("x");
    //pass.setFilterLimits (3.62829, 6.7);
    pass.setFilterLimits ( cableLoc_.x - 0.42, cableLoc_.x + 1.05);
    pass.filter (*output_cloud);
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(output_cloud);
    pass1.setFilterFieldName ("y");
    //pass.setFilterLimits (3.62829, 6.7);
    pass1.setFilterLimits ( cableLoc_.y - 0.8, cableLoc_.y + 0.18);
    pass1.filter (*output_cloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(output_cloud);
    auto inliers = boost::make_shared<pcl::PointIndices>();
    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
    seg.segment(*inliers, *coefficients);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(output_cloud);
    proj.setModelCoefficients(coefficients);
    proj.setIndices(inliers);
    auto cloud_projected = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    proj.filter(*cloud_projected);

    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setDimension(2);
    chull.setComputeAreaVolume(true);
    // Create a Convex Hull representation of the projected inliers
    auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    chull.setInputCloud(cloud_projected);
    chull.reconstruct(*cloud_hull);

    using HullLine = std::tuple<Eigen::Vector3f /* point */, Eigen::Vector3f /* direction */, float>;
    std::vector<HullLine> hull_lines;
    std::transform(cloud_hull->begin(), cloud_hull->end() - 1, cloud_hull->begin() + 1, std::back_inserter(hull_lines),
        [](const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2) {
            const Eigen::Vector3f &v1 = pt1.getVector3fMap();
            const Eigen::Vector3f &v2 = pt2.getVector3fMap();
            const Eigen::Vector3f &dir = v2 - v1;
            return std::make_tuple(v1, dir, dir.squaredNorm());
        }
    );

    pcl::compute3DCentroid(*cloud_hull, centroid);

    ROS_INFO_STREAM("Centroid" << centroid << std::endl);

    geom_point.point.x = centroid(0);
    geom_point.point.y = centroid(1);
    geom_point.point.z = centroid(2);
    geom_point.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), transform);
        //listener.transformPoint(VAL_COMMON_NAMES::PELVIS_TF, geom_point, geom_point);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
    }

    //geom_point1.point.x = transform.getOrigin().x();
    //geom_point1.point.y = transform.getOrigin().y();
    //geom_point1.point.z = centroid(2);
    //geom_point1.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
    //visualize_direction(geom_point.point, geom_point1.point);
    visualize_point(geom_point.point, 0.7, 0.5, 0.0);
    marker_pub_.publish(markers_);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_hull, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);
    output_cloud->points.clear();
    return true;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr max_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//    {
//      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        cloud_cluster->points.push_back (output_cloud->points[*pit]);

//      if (cloud_cluster->points.size() > max_cloud_cluster->points.size())
//        max_cloud_cluster = cloud_cluster;

//    }

//    pcl::compute3DCentroid(*max_cloud_cluster, centroid);
//    ROS_INFO_STREAM("Cloud Size" << max_cloud_cluster->size() << std::endl);
//    ROS_INFO_STREAM("Cluster Indices" << cluster_indices.size() << std::endl);

}

CableDetector::~CableDetector()
{
    pcl_sub_.shutdown();
}

//-0.593457877636
//y: -0.91660118103  point right
//z: 0.772663831711

//x: -0.197707682848
//y: -0.73062390089   point down
//z: 0.794347047806

//x: -0.180317223072
//y: -1.69919061661    point up
//z: 0.807236671448

//x: 0.863122105598
//y: -0.865445196629   point left
//z: 0.776612520218



