#include "val_task2/val_rover_detection.h"
#include "val_common/val_common_names.h"

#define lowerBox_pass_x_min  0.5
#define lowerBox_pass_x_max  10.0
#define lowerBox_pass_y_min -10.0
#define lowerBox_pass_y_max  10.0
#define lowerBox_pass_z_min  1.7
#define lowerBox_pass_z_max  3.0

#define upperBox_pass_x_min -1.5
#define upperBox_pass_x_max  10.0
#define upperBox_pass_y_min -10.0
#define upperBox_pass_y_max  10.0
#define upperBox_pass_z_min  2.5
#define upperBox_pass_z_max  3.0


RoverDetector::RoverDetector(ros::NodeHandle nh)
{
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &RoverDetector::cloudCB, this);
    pcl_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/val_rover/cloud2", 1);

    vis_pub_ = nh.advertise<visualization_msgs::Marker>( "/val_rover/Position", 1 );
    detections_.clear();
    detection_tries_ = 0;
    roverSide_ = ROVER_SIDE::UNKOWN;
}

RoverDetector::~RoverDetector()
{
    pcl_sub_.shutdown();
}

/*
 * NOTE: Applying voxel to the input cloud (refer periodic_snapshotter.cpp)is resulting in the incorrect rover position
 *
 * */

void RoverDetector::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input){

    if (input->data.empty()){
        return;
    }
    ++detection_tries_;

    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr lowerBoxCloud (new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr upperBoxCloud (new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    lowerBoxPassThroughFilter(lowerBoxCloud);
    if (lowerBoxCloud->empty()){
        return;
    }
    segmentation(lowerBoxCloud);
    if (lowerBoxCloud->empty()){
        return;
    }

    upperBoxPassThroughFilter(upperBoxCloud);
    if (upperBoxCloud->empty()){
        return;
    }
    planeDetection(upperBoxCloud);
    if (upperBoxCloud->empty()){
        return;
    }
    segmentation(upperBoxCloud);
    if (upperBoxCloud->empty()){
        return;
    }

    geometry_msgs::Pose pose;
    getPosition(lowerBoxCloud, upperBoxCloud, pose );
    detections_.push_back(pose);

    ros::Time endTime = ros::Time::now();
    std::cout << "Number of detections               = " << detections_.size() << std::endl;
    std::cout << "Number of tries                    = " << detection_tries_ << std::endl;
    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

    pcl::toROSMsg(*upperBoxCloud, output);

    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

    pcl_filtered_pub_.publish(output);

}

void RoverDetector::getPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lowerBoxCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& upperBoxCloud, geometry_msgs::Pose& pose){

    Eigen::Vector4f lowerBoxCentroid;
    pcl::compute3DCentroid(*lowerBoxCloud, lowerBoxCentroid);
    geometry_msgs::Point lowerBoxPosition;
    lowerBoxPosition.x = lowerBoxCentroid(0);
    lowerBoxPosition.y = lowerBoxCentroid(1);
    lowerBoxPosition.z = lowerBoxCentroid(2);

    Eigen::Vector4f upperBoxCentroid;
    pcl::compute3DCentroid(*upperBoxCloud, upperBoxCentroid);
    geometry_msgs::Point upperBoxPosition;
    upperBoxPosition.x = upperBoxCentroid(0);
    upperBoxPosition.y = upperBoxCentroid(1);
    upperBoxPosition.z = upperBoxCentroid(2);

    //    ROS_INFO("Centroid values are X:= %0.2f, Y := %0.2f, Z := %0.2f", upperBoxPosition.x, upperBoxPosition.y, upperBoxPosition.z);

    //  Using Priciple Component Analysis for computing the Orientation of the Panel
    Eigen::Matrix3f covarianceMatrix;
    pcl::computeCovarianceMatrix(*upperBoxCloud, upperBoxCentroid, covarianceMatrix);
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);

    //    std::cout<<"The EigenValues are : " << eigenValues << std::endl;
    //    std::cout<<"The EigenVectors are : " << eigenVectors << std::endl;


    geometry_msgs::Point point1;
    point1.x = eigenVectors.col(2)[0] + upperBoxPosition.x;
    point1.y = eigenVectors.col(2)[1] + upperBoxPosition.y;
    point1.z = eigenVectors.col(2)[2] + upperBoxPosition.z;

    geometry_msgs::Point maxPoint;
    geometry_msgs::Point minPoint;

    maxPoint.x = std::max(upperBoxPosition.x, point1.x);
    maxPoint.y = std::max(upperBoxPosition.y, point1.y);
    maxPoint.z = std::max(upperBoxPosition.z, point1.z);

    minPoint.x = std::min(upperBoxPosition.x, point1.x);
    minPoint.y = std::min(upperBoxPosition.y, point1.y);
    minPoint.z = std::min(upperBoxPosition.z, point1.z);

    float theta = 0;
    float cosTheta = 0;
    float sinTheta = 0;

    cosTheta = (maxPoint.y - minPoint.y)/(sqrt(pow((maxPoint.x - minPoint.x),2) + pow((maxPoint.y - minPoint.y),2) + pow((maxPoint.z - minPoint.z),2)));
    sinTheta= (maxPoint.x - minPoint.x)/(sqrt(pow((maxPoint.x - minPoint.x),2) + pow((maxPoint.y - minPoint.y),2) + pow((maxPoint.z - minPoint.z),2)));
//    sinTheta = sqrt(1 - (pow(cosTheta, 2)));

    double yzSlope = (upperBoxPosition.z - lowerBoxPosition.z)/(upperBoxPosition.y - lowerBoxPosition.y);

    bool noSlope = (fabs((upperBoxPosition.y - point1.y) < 0.01));

    double xySlope = 0.0;

    if(!noSlope){
        xySlope = (upperBoxPosition.x - point1.x)/(upperBoxPosition.y - point1.y);
    }

    if(yzSlope > 0){
        if(!noSlope){
            if(xySlope < 0){
                theta = atan2(sinTheta, cosTheta);
            }
            else if(xySlope > 0){
                theta = atan2(sinTheta, cosTheta)  + 1.5708;
            }
        }
        else if (xySlope == 0){
            theta = atan2(sinTheta, cosTheta);
        }
    }
    else{
        if(!noSlope){
            if(xySlope > 0){
                theta = atan2(sinTheta, cosTheta) * -1.0 ;
            }
            else if(xySlope < 0){
                theta = atan2(sinTheta, cosTheta) * -1.0 - 1.5708;
            }
        }
        else if (xySlope == 0){
            theta = atan2(sinTheta, cosTheta) * -1.0;
        }
    }


    //    ROS_INFO("The Orientation is given by := %0.2f", theta);

    double offset = 6.25;

    pose.position.x = upperBoxPosition.x - (offset*cos(theta));
    pose.position.y = upperBoxPosition.y - (offset*sin(theta));
    pose.position.z = 0.0;

    //    ROS_INFO("Offset values to Footstep Planner are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);
    geometry_msgs::Quaternion quaternion;
    ROS_INFO("slopeyz %.2f, theta %.2f",yzSlope,theta);

    float angle;

    if(yzSlope>0)  {//rover on the left
        angle = theta-1.5708;
        quaternion = tf::createQuaternionMsgFromYaw(theta-1.5708);
        roverSide_ = ROVER_SIDE::LEFT;
    }
    else {
        //rover on the right
        angle = theta+1.5708;
        quaternion = tf::createQuaternionMsgFromYaw(theta+1.5708);
        roverSide_ = ROVER_SIDE::RIGHT;
    }


    float distance_offset = -0.2;
    pose.position.x = pose.position.x + distance_offset * cos(angle);
    pose.position.y = pose.position.y + distance_offset * sin(angle);

    pose.orientation = quaternion;

    visualization_msgs::Marker marker;

    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "Rover Position";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = pose.position.z;

    marker.pose.orientation.x = pose.orientation.x;
    marker.pose.orientation.y = pose.orientation.y;
    marker.pose.orientation.z = pose.orientation.z;
    marker.pose.orientation.w = pose.orientation.w;

    marker.scale.x = 0.6;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(5);

    vis_pub_.publish(marker);

}

bool RoverDetector::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val.clear();
    ret_val = detections_;
    return !ret_val.empty();

}

int RoverDetector::getDetectionTries() const
{
    return detection_tries_;

}

bool RoverDetector::getRoverSide(ROVER_SIDE &side) const
{
    side = roverSide_ ;
    return roverSide_ != ROVER_SIDE::UNKOWN;

}

void RoverDetector::lowerBoxPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(lowerBox_pass_x_min,lowerBox_pass_x_max);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(lowerBox_pass_y_min,lowerBox_pass_y_max);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(lowerBox_pass_z_min,lowerBox_pass_z_max);
    pass_z.filter(*cloud);


}

void RoverDetector::upperBoxPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(upperBox_pass_x_min,upperBox_pass_x_max);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(upperBox_pass_y_min,upperBox_pass_y_max);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(upperBox_pass_z_min,upperBox_pass_z_max);
    pass_z.filter(*cloud);


}

void RoverDetector::planeDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud);

    //  ROS_INFO("Point cloud representing the planar component = %d", (int)cloud->points.size());

}




void RoverDetector::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    int cloudSize = (int)cloud->points.size();

    //  ROS_INFO("Minimum Size = %d", (int)(0.2*cloudSize));
    //  ROS_INFO("Maximum Size = %d", cloudSize);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize ((int)(0.3*cloudSize));
    ec.setMaxClusterSize (cloudSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int numClusters = cluster_indices.size();

    //  ROS_INFO("Number of Clusters  = %d", numClusters);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    int index = 0;
    double x = 0;
    double y = 0;
    double z = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_points (new pcl::PointCloud<pcl::PointXYZ>);

    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        //      ROS_INFO("Number of Points in the Cluster = %d", (int)cloud_cluster->points.size());

        *cloud = *cloud_cluster;

    }

    // ROS_WARN(" %d ", (int)centroid_points->points.size());

}
