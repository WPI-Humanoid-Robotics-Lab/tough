#include "val_task2/array_table_detector.h"

ArrayTableDetector::ArrayTableDetector(ros::NodeHandle nh, geometry_msgs::Point cable_location):nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>){

    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &ArrayTableDetector::cloudCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_table/cloud2", 1, true);
    marker_pub_= nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);

    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    cable_loc_ = cable_location;
}

ArrayTableDetector::~ArrayTableDetector()
{
    pcl_sub_.shutdown();
    pcl_pub_.shutdown();
    marker_pub_.shutdown();
}

bool ArrayTableDetector::planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    ROS_INFO("ArrayTableDetector::planeSegmentation : Detecting table");

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //fetch the cloud which would have table. assuming robot is standing less than 2m away from th etable
    extractCloudOfInterest(input, *output_cloud);
    ROS_INFO("ArrayTableDetector::planeSegmentation : Trimmed Cloud");

    // Downsample the cloud for faster processing
    float leafsize  = 0.05;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leafsize, leafsize, leafsize);
    grid.setInputCloud (output_cloud);
    grid.filter (*output_cloud);

    ROS_INFO("ArrayTableDetector::planeSegmentation : Downsampled the cloud");

    //apply passthrough filter on z
    float min_z = 0.02;
    float max_z = cable_loc_.z - 0.05;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(output_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ( min_z, max_z);
    pass.filter (*output_cloud);

    ROS_INFO("ArrayTableDetector::planeSegmentation : Passthrough filter on z applied");

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(output_cloud);
    auto inliers = boost::make_shared<pcl::PointIndices>();
    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (output_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*output_cloud);

    ROS_INFO("ArrayTableDetector::planeSegmentation : Segmented the plane");

    getLargestCluster(output_cloud, output_cloud);

    ROS_INFO("ArrayTableDetector::planeSegmentation : Got the largest cluster");

    pcl::ConvexHull<pcl::PointXYZ> convHull;
    convHull.setDimension(2);
    convHull.setComputeAreaVolume(true);
    // Create a Convex Hull representation of the projected inliers
    auto cloud_hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    convHull.setInputCloud(output_cloud);
    convHull.reconstruct(*cloud_hull);

    ROS_INFO("ArrayTableDetector::planeSegmentation : Reconstructed the hull");

    pcl::PointXYZ  bb_min, bb_max;
    pcl::getMinMax3D(*cloud_hull, bb_min, bb_max);
    const Eigen::Vector3f &bb_center = (bb_min.getVector3fMap() + bb_max.getVector3fMap())/2;

    geometry_msgs::Point table_center;
    table_center.x = bb_center[0];
    table_center.y = bb_center[1];
    table_center.z = bb_center[2];
    ROS_INFO("ArrayTableDetector::planeSegmentation : Center of table x:%f y:%f z:%f. Area:%f",table_center.x,table_center.y,table_center.z, convHull.getTotalArea());

    Eigen::Vector4f plane_parameters;
    float curvature;
    pcl::computePointNormal(*output_cloud, plane_parameters, curvature);
    ROS_INFO("ArrayTableDetector::planeSegmentation : Normal Params : %f %f %f %f", plane_parameters[0], plane_parameters[1], plane_parameters[2], plane_parameters[3]);

    float cos_theta = plane_parameters(0)/std::sqrt(std::pow(plane_parameters(0),2) + std::pow(plane_parameters(1),2) );
    float sin_theta = plane_parameters(1)/std::sqrt(std::pow(plane_parameters(0),2) + std::pow(plane_parameters(1),2) );
    float theta = std::atan2(sin_theta, cos_theta);
    geometry_msgs::Pose pose;
    ROS_INFO("ArrayTableDetector::planeSegmentation : Yaw angle : %f", theta);
    pose.position.x = table_center.x;
    pose.position.y = table_center.y;
    pose.position.z = 0;

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
    pose.orientation = quaternion;

    robot_state_->transformPose(pose, pose, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    ROS_INFO_STREAM("acos " << tf::getYaw(pose.orientation) << std::endl);

    if (!(tf::getYaw(pose.orientation) < M_PI_2 && tf::getYaw(pose.orientation) > -M_PI_2))
    {
        pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose.orientation) - M_PI);
    }
    robot_state_->transformPose(pose, pose, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
    ROS_INFO_STREAM("acos1 " << tf::getYaw(pose.orientation) << std::endl);
    pose.position.x = table_center.x + TABLE_OFFSET * std::cos(tf::getYaw(pose.orientation));
    pose.position.y = table_center.y + TABLE_OFFSET * std::sin(tf::getYaw(pose.orientation));

    detections_.push_back(pose);
    visualizePose(pose);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*output_cloud, output);
    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    pcl_pub_.publish(output);

    output_cloud->points.clear();
    return true;

}

void ArrayTableDetector::extractCloudOfInterest(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ> &output)
{
    geometry_msgs::Point cableLocation;
    cableLocation.x = cable_loc_.x;
    cableLocation.y = cable_loc_.y;
    robot_state_->transformPoint(cableLocation, cableLocation, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0]=0;
    minPoint[1]=-2;
    minPoint[2]=-1;

    maxPoint[0]=cableLocation.x;
    maxPoint[1]=2;
    maxPoint[2]=0.5;
    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0]=pelvisPose.position.x;
    boxTranslatation[1]=pelvisPose.position.y;
    boxTranslatation[2]=pelvisPose.position.z;
    Eigen::Vector3f boxRotation;
    boxRotation[0]=0;  // rotation around x-axis
    boxRotation[1]=0;  // rotation around y-axis
    boxRotation[2]= tf::getYaw(pelvisPose.orientation);  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.


    pcl::CropBox<pcl::PointXYZ> box_filter;
    std::vector<int> indices;
    indices.clear();
    box_filter.setInputCloud(input);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setTranslation(boxTranslatation);
    box_filter.setRotation(boxRotation);
    box_filter.setNegative(false);
    box_filter.filter(output);
}

void ArrayTableDetector::getLargestCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (input);
    int cloudSize = (int)input->points.size();
    //  ROS_INFO("Minimum Size = %d", (int)(0.2*cloudSize));
    //  ROS_INFO("Maximum Size = %d", cloudSize);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize ((int)(0.51*cloudSize));
    ec.setMaxClusterSize (cloudSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input);
    ec.extract (cluster_indices);

    std::vector<int>::const_iterator pit;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO_STREAM("cluster indices size " << cluster_indices[0].indices.size() << std::endl);
    if (cluster_indices.size() == 1)
    {
        for(pit = cluster_indices[0].indices.begin(); pit != cluster_indices[0].indices.end(); pit++) {
            cloud_cluster->points.push_back(input->points[*pit]);
        }
    }
    ROS_INFO("ArrayTableDetector::getLargestCluster : Number of points in the cluster = %d", (int)cloud_cluster->points.size());
    *output = *cloud_cluster;
}

void ArrayTableDetector::cloudCB(const sensor_msgs::PointCloud2::Ptr& input)
{
    if (input->data.empty())
        return;
    mtx_.lock();
    pcl::fromROSMsg(*input, *cloud_);
    mtx_.unlock();
    planeSegmentation(cloud_);
}


void ArrayTableDetector::visualizePose(const geometry_msgs::Pose &pose, double r, double g, double b){

    static int id = 0;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "array_table";
    marker.id = id++;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose = pose;

    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    marker_pub_.publish(marker);
}

bool ArrayTableDetector::getDetections(std::vector<geometry_msgs::Pose> &output)
{
    output = detections_;
    return !detections_.empty();
}

void ArrayTableDetector::clearDetections()
{
    mtx_.lock();
    detections_.clear();
    mtx_.unlock();
}
