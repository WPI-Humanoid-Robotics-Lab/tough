#include "val_solar_detection/val_solar_detection.h"

plane::plane(ros::NodeHandle nh, geometry_msgs::Pose rover_loc)
{


  current_state_ = RobotStateInformer::getRobotStateInformer(nh);


  pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &plane::cloudCB, this);

//    pcl_sub =  nh.subscribe("/field/octomap_point_cloud_centers", 10, &plane::cloudCB, this);
  pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_plane/cloud2", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "/val_solar/visualization_marker", 1 );
  rover_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_task2/rover_cloud",1);

  rover_loc_ = rover_loc;

}


void plane::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
{
    geometry_msgs::Pose location;
    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(*input, *cloud);

   roverremove(cloud);
    PassThroughFilter(cloud);

    ROS_INFO("pub %d",(int)cloud->points.size());

    planeDetection(cloud);
    getPosition(cloud,location);
    pcl::toROSMsg(*cloud,output);
    output.header.frame_id="world";
    pcl_filtered_pub.publish(output);

    ros::Time endTime = ros::Time::now();
    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;



}
void plane::roverremove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    tf::Quaternion quat(rover_loc_.orientation.x,rover_loc_.orientation.y,rover_loc_.orientation.z,rover_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);
    double theta = yaw;

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0]=0;
    minPoint[1]=-2;
    minPoint[2]=0;

    maxPoint[0]=9;
    maxPoint[1]=+2;
    maxPoint[2]=3;
    Eigen::Vector3f boxTranslatation;
         boxTranslatation[0]=rover_loc_.position.x;
         boxTranslatation[1]=rover_loc_.position.y;
         boxTranslatation[2]=0;
    Eigen::Vector3f boxRotation;
         boxRotation[0]=0;  // rotation around x-axis
         boxRotation[1]=0;  // rotation around y-axis
         boxRotation[2]=theta;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.


    pcl::CropBox<pcl::PointXYZ> box_filter;
    pcl::PointCloud<pcl::PointXYZ> rover_cloud;
    std::vector<int> indices;

    indices.clear();
    box_filter.setInputCloud(cloud);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setTranslation(boxTranslatation);
    box_filter.setRotation(boxRotation);
    box_filter.setNegative(true);
    box_filter.filter(*cloud);


    sensor_msgs::PointCloud2 rover_output;
    box_filter.setNegative(false);
    box_filter.filter(rover_cloud);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(rover_cloud,rover_output);
    output.header.frame_id="world";
    rover_cloud_pub.publish(rover_output);

//    box_filter.filter(indices);


}

void plane::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


    // need to use robot_stateinformer

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-2,10);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-10,10);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(1,1.68);
    pass_z.filter(*cloud);

}

void plane::planeDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

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




void plane::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose){

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
    ROS_INFO("slope is %f",slope);


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

    double offset = -1.2;
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

    vis_pub.publish(marker);


}

