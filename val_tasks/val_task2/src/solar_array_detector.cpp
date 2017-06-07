#include "val_task2/solar_array_detector.h"

CoarseArrayDetector::CoarseArrayDetector(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &CoarseArrayDetector::cloudCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_solar_plane/cloud2", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_normal",1);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

CoarseArrayDetector::~CoarseArrayDetector()
{
    pcl_sub_.shutdown();
    pcl_pub_.shutdown();
    marker_pub_.shutdown();
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

bool CoarseArrayDetector::getArrayPosition(const geometry_msgs::Pose2D& rover_pose)
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

    if (!output_cloud->empty())
        array_detected = true;

    if(output_cloud->empty())
        ROS_ERROR("CoarseArrayDetector::Dandanakka not done!!!!");

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
}

void CoarseArrayDetector::normalSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    //box filter
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

    // Downsample the cloud for faster processing
    float leafsize  = 0.02;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leafsize, leafsize, leafsize);
    grid.setInputCloud (temp_cloud);
    grid.filter (*temp_cloud);

    // Normals estimation
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

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*temp_cloud);

    if(temp_cloud->empty()){
        ROS_ERROR("There are no normals in the point cloud!");
        return;
    }

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
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);

    //find the clusters
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

    //Iterate to find the array cluster
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
        minPoint[0] = cloudCentroid(0) - 1.7;
        minPoint[1] = cloudCentroid(1) -2;
        minPoint[2] = pelvisPose.position.z + 0.5;

        maxPoint[0] = cloudCentroid(0) + 3.0;
        maxPoint[1] = cloudCentroid(1) + 2.0;
        maxPoint[2] = 2.0;

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setInputCloud(cloud_);
        box_filter.setMin(minPoint);
        box_filter.setMax(maxPoint);
        box_filter.setNegative(false);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        box_filter.filter(*temp_cloud);

        //Downsample cloud
        float leafsize  = 0.05;
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize (leafsize, leafsize, leafsize);
        grid.setInputCloud (temp_cloud);
        grid.filter (*temp_cloud);

        if (temp_cloud->empty())
            continue;

//        sensor_msgs::PointCloud2 output1;
//        pcl::toROSMsg(*temp_cloud, output1);
//        output1.header.frame_id = "world";
//        pcl_pub_.publish(output1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        if (planeDetection(temp_cloud, temp_cloud1))
            *output = *temp_cloud1;
        //ROS_INFO_STREAM("output size " << output->points.size()<< std::endl);
    }
}

bool CoarseArrayDetector::planeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    if (input->points.size() < 20)
        return false;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.25);
    seg.setInputCloud (input);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.segment (*inliers, *coefficients);

    if (coefficients->values.empty() || inliers->indices.empty())
        return false;

    if (coefficients->values[2] < 0)
    {
      coefficients->values[0] = -coefficients->values[0];
      coefficients->values[1] = -coefficients->values[1];
      coefficients->values[2] = -coefficients->values[2];
    }

    float cos_theta1 = coefficients->values[2]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) + std::pow(coefficients->values[2],2));
    float sin_theta1 = std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2))/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) + std::pow(coefficients->values[2],2));
    float theta1 = std::atan2(sin_theta1, cos_theta1);

    ROS_INFO_STREAM("theta1 "<< theta1 << "\t" <<coefficients->values[2] << std::endl);

    if (theta1 < 0.72 || theta1 > 1.00)
        return false;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*output);

    float leafsize  = 0.05;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leafsize, leafsize, leafsize);
    grid.setInputCloud (output);
    grid.filter (*output);

    Eigen::Vector4f cloudCentroid;
    pcl::compute3DCentroid(*output, cloudCentroid);

    double t = -(cloudCentroid(2)) / double(coefficients->values[2]);
    float cos_theta = coefficients->values[0]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) );
    float sin_theta = coefficients->values[1]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) );
    float theta = std::atan2(sin_theta, cos_theta);

    geometry_msgs::Pose pose;
    pose.position.x = cloudCentroid(0) + ARRAY_OFFSET * t * coefficients->values[0];
    pose.position.y = cloudCentroid(1) + ARRAY_OFFSET * t * coefficients->values[1];
    pose.position.z = 0;

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
    pose.orientation = quaternion;

    visualizePose(pose);
    marker_pub_.publish(markers_);
    return true;

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
