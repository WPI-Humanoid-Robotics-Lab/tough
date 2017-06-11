#include "val_task2/solar_array_detector.h"

CoarseArrayDetector::CoarseArrayDetector(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &CoarseArrayDetector::cloudCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_array_cloud/cloud2", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    rover_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/block_map", 1, true);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

CoarseArrayDetector::~CoarseArrayDetector()
{
    pcl_sub_.shutdown();
    pcl_pub_.shutdown();
    marker_pub_.shutdown();
    rover_cloud_pub_.shutdown();
}

void CoarseArrayDetector::cloudCB(const sensor_msgs::PointCloud2::Ptr input)
{
    if (input->data.empty())
        return;
    //add mutex
    mtx_.lock();
    pcl::fromROSMsg(*input, *cloud_);
    mtx_.unlock();
}

bool CoarseArrayDetector::getArrayPosition(const geometry_msgs::Pose2D& rover_pose, geometry_msgs::Pose& array_pose)
{
    markers_.markers.clear();
    bool array_detected = false;
    rover_pose_ = rover_pose;

    if (cloud_->empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //copy cloud into local variable
    mtx_.lock();
    input_cloud = cloud_;
    mtx_.unlock();

    if (input_cloud->empty())
        return false;

    ROS_INFO("CoarseArrayDetector::Removing Rover from Point Cloud");
    roverRemove(input_cloud, out_cloud1);

    if (out_cloud1->empty())
        return false;

    ROS_INFO("CoarseArrayDetector::Computing Normals");
    normalSegmentation(out_cloud1, out_cloud2);

    if (out_cloud2->empty())
        return false;

    ROS_INFO("CoarseArrayDetector::Finding Array Cluster");
    findArrayCluster(out_cloud2, output_cloud);

    if (!output_cloud->empty()){
        array_pose = pose_;
        array_detected = true;
        ROS_INFO("CoarseArrayDetector::Found the Coarse Array Pose!");
    }

    //publish the output cloud for visualization
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*output_cloud, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);

    return array_detected;
}

void CoarseArrayDetector::roverRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    //box filter
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0] = 0.0;
    minPoint[1] = -2.0;
    minPoint[2] = 0.0;

    maxPoint[0] = 9.0;
    maxPoint[1] = 2.0;
    maxPoint[2] = 3.0;

    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0] = rover_pose_.x;
    boxTranslatation[1] = rover_pose_.y;
    boxTranslatation[2] = 0.1;  // to remove the points belonging to the walkway

    Eigen::Vector3f boxRotation;
    boxRotation[0] = 0.0;  // rotation around x-axis
    boxRotation[1] = 0.0;  // rotation around y-axis
    boxRotation[2] = rover_pose_.theta;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

    pcl::CropBox<pcl::PointXYZ> box_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rover_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.setInputCloud(input);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setTranslation(boxTranslatation);
    box_filter.setRotation(boxRotation);
    box_filter.setNegative(false);
    box_filter.filter(*rover_cloud);

    if (rover_cloud->empty())
        return;

    //projecting rover cloud onto xy plane
    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
    coefficients->header.frame_id = "world";
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(rover_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*rover_cloud);

    // Widen cloud in +/- y to make sure val doesn't walk into the side of it
    Eigen::Affine3f rover_shift_l, rover_shift_r;
    const auto rover_align = Eigen::AngleAxisf(rover_pose_.theta, Eigen::Vector3f::UnitZ());
    rover_shift_l.linear() = Eigen::Matrix3f::Identity();
    rover_shift_l.translation() = rover_align * Eigen::Vector3f(-0.2, -0.4, 0);
    rover_shift_r.linear() = Eigen::Matrix3f::Identity();
    rover_shift_r.translation() = rover_align * Eigen::Vector3f(-0.2, 0.4, 0);

    pcl::PointCloud<pcl::PointXYZ> rover_shifted_l, rover_shifted_r;
    pcl::transformPointCloud(*rover_cloud, rover_shifted_l, rover_shift_l);
    pcl::transformPointCloud(*rover_cloud, rover_shifted_r, rover_shift_r);
    rover_shifted_l += rover_shifted_r;

    box_filter.setNegative(true);
    box_filter.filter(*output);

    sensor_msgs::PointCloud2 rover_output;
    pcl::toROSMsg(rover_shifted_l, rover_output);
    rover_output.header.frame_id = "world";
    rover_cloud_pub_.publish(rover_output);
}

void CoarseArrayDetector::normalSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    //box filter to limit search space for normals
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0] = -2.0;
    minPoint[1] = -4.0;
    minPoint[2] = -pelvisPose.position.z + 0.1;

    maxPoint[0] = 8.0;
    maxPoint[1] = 4.0;
    maxPoint[2] = 2.0;

    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0] = pelvisPose.position.x;
    boxTranslatation[1] = pelvisPose.position.y;
    boxTranslatation[2] = pelvisPose.position.z;

    Eigen::Vector3f boxRotation;
    boxRotation[0] = 0;  // rotation around x-axis
    boxRotation[1] = 0;  // rotation around y-axis
    boxRotation[2] = tf::getYaw(pelvisPose.orientation);

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setInputCloud(input);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setTranslation(boxTranslatation);
    box_filter.setRotation(boxRotation);
    box_filter.setNegative(false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.filter(*temp_cloud);

    //Downsample the cloud
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> oct(0.02);
    oct.setInputCloud(temp_cloud);
    oct.addPointsFromInputCloud();

    pcl::PointCloud<pcl::PointXYZ>::VectorType pts;
    oct.getVoxelCentroids(pts);

    temp_cloud->clear();
    for (const auto &pt : pts) {
        temp_cloud->push_back(pcl::PointXYZ());
        temp_cloud->back().x = pt.x;
        temp_cloud->back().y = pt.y;
        temp_cloud->back().z = pt.z;
    }

    // Find normals to all the points
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (temp_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.compute (*cloud_normals);

    std::vector<int> index;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, index);

    auto inliers = boost::make_shared<pcl::PointIndices>();
    inliers->indices = index;
    inliers->header = cloud_->header;

    //Remove points with normal as NaN
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*temp_cloud);

    if(temp_cloud->empty()){
        ROS_ERROR("There are no normals in the point cloud!");
        return;
    }

    // Create new cloud whose normals are perpendicular to world z axis
    for(size_t i = 0; i < cloud_normals->points.size(); i++){
        if(fabs(cloud_normals->points[i].normal_z) < 0.01)
            output->points.push_back(temp_cloud->points[i]);
    }

    if(output->empty()){
        ROS_ERROR("There are no points above the threshold to compute normal");
        return;
    }
}

void CoarseArrayDetector::findArrayCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    int counter = 0;
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    //find all the candidate array clusters
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (input);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize ((int)(0.10 * (input->points.size())));  //10% to make sure minimize the number of clusters
    ec.setMaxClusterSize ((int)(input->points.size()));
    ec.setSearchMethod (tree);
    ec.setInputCloud (input);
    ec.extract (cluster_indices);

    //Iterate to find the correct array cluster
    //For each cluster,
    //Steps : Find the centroid, then box filter w.r.t centroid, detecting slant plane, appending detected slant planes

    ROS_INFO_STREAM("Cluster size " << cluster_indices.size());
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        auto inliers = boost::make_shared<pcl::PointIndices>();
        inliers->indices = it->indices;
        inliers->header = cloud_->header;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (input);
        extract.setIndices (inliers);
        extract.setNegative (false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter (*cloud_cluster);

        Eigen::Vector4f cloudCentroid;
        pcl::compute3DCentroid(*cloud_cluster, cloudCentroid);

        //box filter
        Eigen::Vector4f minPoint;
        Eigen::Vector4f maxPoint;
        minPoint[0] = cloudCentroid(0) - 2.5;
        minPoint[1] = cloudCentroid(1) -2.5;
        minPoint[2] = pelvisPose.position.z + 0.5;

        maxPoint[0] = cloudCentroid(0) + 3.0;
        maxPoint[1] = cloudCentroid(1) + 2.5;
        maxPoint[2] = 2.0;

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud_);
        box_filter.setMin(minPoint);
        box_filter.setMax(maxPoint);
        box_filter.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        box_filter.filter(*temp_cloud);

        if (temp_cloud->empty())
            continue;

        //check and append if this plane matches with known slant plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        if (planeDetection(temp_cloud, temp_cloud1, coefficients ))
        {
            if (counter == 0)
            {
                *output = *temp_cloud1;
                counter++;
                continue;
            }
            *output += *temp_cloud1;
        }
        //ROS_INFO_STREAM("output size " << output->points.size()<< std::endl);
    }

    //Downsample cloud
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> oct(0.05);
    oct.setInputCloud(output);
    oct.addPointsFromInputCloud();
    pcl::PointCloud<pcl::PointXYZ>::VectorType pts;
    oct.getVoxelCentroids(pts);
    output->clear();
    for (const auto &pt : pts) {
        output->push_back(pcl::PointXYZ());
        output->back().x = pt.x;
        output->back().y = pt.y;
        output->back().z = pt.z;
    }

    //Find the plane eqaution of appended cluster
     ROS_INFO_STREAM("output size " << output->points.size()<< std::endl);
    if(!planeDetection(output, output, coefficients))
    {
        output->points.clear();
        return;
    }
    ROS_INFO_STREAM("output size " << output->points.size()<< std::endl);

    //Get the coarse pose near array and visualize it
    standPose(output, coefficients);

}

bool CoarseArrayDetector::planeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, pcl::ModelCoefficients::Ptr coefficients)
{
    //Find plane equation for the input cloud
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.25);
    seg.setInputCloud (input);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.segment (*inliers, *coefficients);

    if (coefficients->values.empty() || inliers->indices.empty())
        return false;

    //Make sure that the normal of the plane is always pointing upwards (useful when giving offsets)
    if (coefficients->values[2] < 0)
    {
      coefficients->values[0] = -coefficients->values[0];
      coefficients->values[1] = -coefficients->values[1];
      coefficients->values[2] = -coefficients->values[2];
    }

    //To find angle of the normal w.r.t z axis of world
    float cos_theta1 = coefficients->values[2]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) + std::pow(coefficients->values[2],2));
    float sin_theta1 = std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2))/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) + std::pow(coefficients->values[2],2));
    float theta1 = std::atan2(sin_theta1, cos_theta1);

    ROS_INFO_STREAM("theta1 "<< theta1 << "\t" <<coefficients->values[2] << std::endl);

    //Check if plane is at known slant angle of the array
    if (theta1 < 0.72 || theta1 > 1.00)
        return false;

    //Extract the points and return cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*output);
    return true;
}

void CoarseArrayDetector::standPose(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::ModelCoefficients::Ptr coefficients)
{
    Eigen::Vector4f cloudCentroid;
    pcl::compute3DCentroid(*input, cloudCentroid);

    //Find the point on line passing thorugh centroid in direction of plane normal at z = 0
    double t = -(cloudCentroid(2)) / double(coefficients->values[2]);
    float cos_theta = coefficients->values[0]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) );
    float sin_theta = coefficients->values[1]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) );
    float theta = std::atan2(sin_theta, cos_theta);

    //Find the corresponding pose
    pose_.position.x = cloudCentroid(0) + ARRAY_OFFSET * t * coefficients->values[0];
    pose_.position.y = cloudCentroid(1) + ARRAY_OFFSET * t * coefficients->values[1];
    pose_.position.z = 0;

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
    pose_.orientation = quaternion;

    //Visualize Pose as marker array
    visualizePose(pose_);
    marker_pub_.publish(markers_);
}

void CoarseArrayDetector::visualizePose(const geometry_msgs::Pose &pose)
{
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
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    markers_.markers.push_back(marker);
}
