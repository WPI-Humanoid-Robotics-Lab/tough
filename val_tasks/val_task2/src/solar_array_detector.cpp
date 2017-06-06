#include "val_task2/solar_array_detector.h"

ArrayDetector::ArrayDetector(ros::NodeHandle& nh) : nh_(nh), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    pcl_sub_ = nh_.subscribe("/field/assembled_cloud2", 10, &ArrayDetector::cloudCB, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/val_solar_plane/cloud2", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_normal",1);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

ArrayDetector::~ArrayDetector()
{
    pcl_sub_.shutdown();
    pcl_pub_.shutdown();
    marker_pub_.shutdown();
}

void ArrayDetector::cloudCB(const sensor_msgs::PointCloud2::Ptr input)
{
    if (input->data.empty())
        return;
    //add mutex
    mtx_.lock();
    pcl::fromROSMsg(*input, *cloud_);
    mtx_.unlock();
}

bool ArrayDetector::getArrayPosition(const geometry_msgs::Pose2D& rover_pose)
{
    markers_.markers.clear();
    bool array_detected = false;

    rover_pose_ = rover_pose;

    if (cloud_->empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //copy cloud into local variable
    mtx_.lock();
    input_cloud = cloud_;
    mtx_.unlock();
    ROS_INFO_STREAM("cloud header" << cloud_->header << std::endl);


    //Segment the plane and pass through filter to narrow the filter region to the stairs
    if (input_cloud->empty())
        return false;

    ROS_INFO("Removing Rover from Point Cloud");

    roverRemove(input_cloud, out_cloud1);

    if (out_cloud1->empty())
        return false;
    ROS_INFO("Computing Normals");
    boxFilter(out_cloud1 , out_cloud2);

    if (out_cloud2->empty())
        return false;
    ROS_INFO("Plane Detection");
    planeDetection(out_cloud2, out_cloud3);

    if (!out_cloud3->empty())
        array_detected = true;
//    ROS_INFO("Finding Largest Cluster");
//    findLargestCluster(out_cloud3, output_cloud);

//    if (!output_cloud->empty())
//        array_detected = true;

    //publish the output cloud for visualization
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*out_cloud3, output);
    output.header.frame_id = "world";
    pcl_pub_.publish(output);

    return array_detected;
}

void ArrayDetector::boxFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    geometry_msgs::Pose pelvisPose;
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0] = -2.0;
    minPoint[1] = -8.0;
    minPoint[2] = 0.5;
    //minPoint.normalize();

//    ROS_INFO_STREAM("pelvis pose" << pelvisPose.position << std::endl);
//    ROS_INFO_STREAM("pelvis orientation yaw" << tf::getYaw(pelvisPose.orientation) << std::endl);
//    ROS_INFO_STREAM("pelvis orientation quaternion" << pelvisPose.orientation << std::endl);



    maxPoint[0] = 8.0;//(pelvisPose.position.x - rover_pose_.x) * 5;
    maxPoint[1] = 8.0;//(pelvisPose.position.y - rover_pose_.y) * 5;
    maxPoint[2] = 2.0;
    //maxPoint.normalize();

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
    box_filter.filter(*output);
}


void ArrayDetector::planeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  Eigen::Vector4f cloudCentroid;


  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.25);
  seg.setInputCloud (input);
  seg.segment (*inliers, *coefficients);
  ROS_INFO_STREAM(" coefficients" << coefficients->values[0] <<"\t" << coefficients->values[1] <<"\t" << coefficients->values[2] <<"\t" << coefficients->values[3] <<"\t" << std::endl);

  if (coefficients->values[2] < 0)
  {
      coefficients->values[0] = -coefficients->values[0];
      coefficients->values[1] = -coefficients->values[1];
      coefficients->values[2] = -coefficients->values[2];

  }

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


  pcl::compute3DCentroid(*output, cloudCentroid);

  double t = -(cloudCentroid(2)) / double(coefficients->values[2]);


  float cos_theta = coefficients->values[0]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) );
  float sin_theta = coefficients->values[1]/std::sqrt(std::pow(coefficients->values[0],2) + std::pow(coefficients->values[1],2) );
  float theta = std::atan2(sin_theta, cos_theta);
  geometry_msgs::Pose pose;

  pose.position.x = cloudCentroid(0) + t * coefficients->values[0];
  pose.position.y = cloudCentroid(1) + t * coefficients->values[1];
  pose.position.z = 0;


  geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
  pose.orientation = quaternion;
  ROS_INFO_STREAM("t " << t << std::endl << "theta" << theta <<std::endl << "pose" << pose <<std::endl);

  visualizePose(pose);

  marker_pub_.publish(markers_);

}


void ArrayDetector::normalSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Downsample the cloud for faster processing
    float leafsize  = 0.00002;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (leafsize, leafsize, leafsize);
    grid.setInputCloud (input);
    grid.filter (*temp_cloud);

    // Normals estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (temp_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
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


void ArrayDetector::roverRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

    double theta;
    theta = rover_pose_.theta;

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0]=0;
    minPoint[1]=-2;
    minPoint[2]=0;

    maxPoint[0]=9;
    maxPoint[1]=+2;
    maxPoint[2]=3;
    Eigen::Vector3f boxTranslatation;
         boxTranslatation[0]=rover_pose_.x;
         boxTranslatation[1]=rover_pose_.y;
         boxTranslatation[2]=0.1;  // to remove the points belonging to the walkway
    Eigen::Vector3f boxRotation;
         boxRotation[0]=0;  // rotation around x-axis
         boxRotation[1]=0;  // rotation around y-axis
         boxRotation[2]=theta;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.

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
    const auto rover_align = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());

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

void ArrayDetector::findLargestCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (input);
    int cloudSize = (int)input->points.size();
    //  ROS_INFO("Minimum Size = %d", (int)(0.2*cloudSize));
    //  ROS_INFO("Maximum Size = %d", cloudSize);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize ((int)(0.40*cloudSize));
    ec.setMaxClusterSize (cloudSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input);
    ec.extract (cluster_indices);

    ROS_INFO_STREAM("cluster indices size " << cluster_indices[0].indices.size() << std::endl);
    if (cluster_indices.size() == 1)
    {
        auto inliers = boost::make_shared<pcl::PointIndices>();
        inliers->indices = cluster_indices[0].indices;
        inliers->header = cloud_->header;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (input);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*output);

    }
    //ROS_INFO("ArrayDetector::findLargestCluster : Number of points in the cluster = %d", (int)cloud_cluster->points.size());

}

void ArrayDetector::visualizePose(const geometry_msgs::Pose &pose){

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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    markers_.markers.push_back(marker);
}
