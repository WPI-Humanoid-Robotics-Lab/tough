#include "val_task1/panel_detection.h"
#include "val_common/val_common_names.h"

panel_detector::panel_detector(ros::NodeHandle &nh)
{
  pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &panel_detector::cloudCB, this);
  pcl_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/val_filter/filteredPointCloud", 1);

  vis_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
  vis_plane_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_plane_vector", 1 );

  num_of_detections_ = 0;
}
short panel_detector::getNumOfDetections()
{
    return num_of_detections_;
}

void panel_detector::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val = detections;
}


void panel_detector::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input){

    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  sensor_msgs::PointCloud2 output;

  pcl::fromROSMsg(*input, *cloud);

  passThroughFilter(cloud);
  panelSegmentation(cloud);
  segmentation(cloud);


  geometry_msgs::Pose pose;
  getPosition(cloud, pose);
  detections.push_back(pose);
  ros::Time endTime = ros::Time::now();

//  std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

  pcl::toROSMsg(*cloud, output);

  output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

  pcl_filtered_pub_.publish(output);

  ++num_of_detections_;
}

void panel_detector::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(1,4);
  pass_x.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-2,2);
  pass_y.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.7,0.87);
  pass_z.filter(*cloud);



}

void panel_detector::panelSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.008);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud);

//  ROS_INFO("Point cloud representing the planar component = %d", (int)cloud->points.size());

}

void panel_detector::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose){

    Eigen::Vector4f centroid;
//  Calculating the Centroid of the Panel Point cloud
    pcl::compute3DCentroid(*cloud, centroid);
    pose.position.x = centroid(0);
    pose.position.y = centroid(1);
    pose.position.z = centroid(2);

//  Using Priciple Component Analysis for computing the Orientation of the Panel
    Eigen::Matrix3f covarianceMatrix;
    pcl::computeCovarianceMatrix(*cloud, centroid, covarianceMatrix);
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);

//    std::cout<<"The EigenValues are : " << eigenValues << std::endl;
//    std::cout<<"The EigenVectors are : " << eigenVectors << std::endl;

    ROS_INFO("Centroid values are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);

    geometry_msgs::Point point1;
    point1.x = eigenVectors.col(2)[0] + pose.position.x;
    point1.y = eigenVectors.col(2)[1] + pose.position.y;
    point1.z = eigenVectors.col(2)[2] + pose.position.z;

    geometry_msgs::Point maxPoint;
    geometry_msgs::Point minPoint;

    maxPoint.x = std::max(pose.position.x, point1.x);
    maxPoint.y = std::max(pose.position.y, point1.y);
    maxPoint.z = std::max(pose.position.z, point1.z);

    minPoint.x = std::min(pose.position.x, point1.x);
    minPoint.y = std::min(pose.position.y, point1.y);
    minPoint.z = std::min(pose.position.z, point1.z);

    geometry_msgs::Point point2;
    point2.x = eigenVectors.col(0)[0] + pose.position.x;
    point2.y = eigenVectors.col(0)[1] + pose.position.y;
    point2.z = eigenVectors.col(0)[2] + pose.position.z;

    double slope = (pose.position.z - point2.z)/(pose.position.y - point2.y);

    float theta = 0;
    float cosTheta = 0;
    float sinTheta = 0;

    cosTheta = (maxPoint.y - minPoint.y)/(sqrt(pow((maxPoint.x - minPoint.x),2) + pow((maxPoint.y - minPoint.y),2) + pow((maxPoint.z - minPoint.z),2)));
    sinTheta = sqrt(1 - (pow(cosTheta, 2)));
    double value = sinTheta/cosTheta;
    if(slope > 0){

        theta = atan2(sinTheta, cosTheta) * -1.0;

    }
    else{

        theta = atan2(sinTheta, cosTheta);

    }

    ROS_INFO("The Orientation is given by := %0.2f", theta);

    double offset = 1.0;
    pose.position.x = pose.position.x - (offset*cos(theta));
    pose.position.y = pose.position.y - (offset*sin(theta));
    pose.position.z = 0.0;

    ROS_INFO("Offset values to Footstep Planner are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);

    pose.orientation = quaternion;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Center of the Panel";
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


void panel_detector::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

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

int main(int argc, char** argv){
  ros::init(argc, argv, "panel_detector");
  ros::NodeHandle nh;
  ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("/valkyrie/goal",1);
  panel_detector obj(nh);
  int NUM_SAMPLES = 4;

  while(ros::ok() && obj.getNumOfDetections() < NUM_SAMPLES){
    ros::spinOnce();
  }

  std::vector<geometry_msgs::Pose> poses;
  obj.getDetections(poses);

  std::sort(poses.begin(),poses .end(), poseComparator);

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
  goal.pose = poses[NUM_SAMPLES/2 -1];
  goalPub.publish(goal);

  ros::Duration(1).sleep();
  ROS_INFO("Exiting panel detector node");

  return 0;


}
