#include "val_task2/val_solar_detection.h"
#define solar_pass_x_min -5.0
#define solar_pass_x_max  10.0
#define solar_pass_y_min -10.0
#define solar_pass_y_max  10.0
#define solar_pass_z_min  1.0
#define solar_pass_z_max  1.68

/* x axis : -10 to 10
 * y axis : -10 to 10
 * z axis : 1 to 1.68
 * */


RoverBlocker::RoverBlocker(ros::NodeHandle nh, geometry_msgs::Pose rover_loc)
{
  pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &RoverBlocker::cloudCB, this);
  pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_plane/cloud2", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker>( "/val_solar/visualization_marker", 1 );
  rover_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/block_map",1);

  rover_loc_ = rover_loc;
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
  detection_tries_ = 0;
  detections_.clear();
}
RoverBlocker::~RoverBlocker()
{
      pcl_sub.shutdown();
}

bool RoverBlocker::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val.clear();
    ret_val = detections_;
    return !ret_val.empty();

}
int RoverBlocker::getDetectionTries() const
{
    return detection_tries_;

}


void RoverBlocker::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
{

    if (input->data.empty()){
        return;
    }
    ++detection_tries_;


    geometry_msgs::Pose location;
    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(*input, *cloud);

   roverremove(cloud);
   PassThroughFilter(cloud);

//    ROS_INFO("pub %d",(int)cloud->points.size());

    planeDetection(cloud);
    getPosition(cloud,location);
    detections_.push_back(location);

    pcl::toROSMsg(*cloud,output);
    output.header.frame_id="world";
    pcl_filtered_pub.publish(output);

    ros::Time endTime = ros::Time::now();
    std::cout << "Number of detections               = " << detections_.size() << std::endl;
    std::cout << "Number of tries                    = " << detection_tries_ << std::endl;
    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

}

void RoverBlocker::roverremove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    tf::Quaternion quat(rover_loc_.orientation.x,rover_loc_.orientation.y,rover_loc_.orientation.z,rover_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);

//    double theta = yaw-1.5708; // rover right of walkway
    double theta = yaw+1.5708;   //left of walkway

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
         boxTranslatation[2]=0.1;  // to remove the points belonging to the walkway
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



    sensor_msgs::PointCloud2 rover_output;
    box_filter.setNegative(false);
    box_filter.filter(rover_cloud);

    // brute force projecting the points into xy plane(z=0)
    for (auto i = rover_cloud.points.begin() ;i!=rover_cloud.points.end();++i)
    {
        (i)->z = 0;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(rover_cloud,rover_output);
    //ROS_INFO("rover cloud size %d",(int)rover_cloud.points.size());
    output.header.frame_id="world";
    rover_cloud_pub.publish(rover_output);


    box_filter.setNegative(true);
    box_filter.filter(*cloud);
//    box_filter.filter(indices);


}

void RoverBlocker::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


    float min_x,min_y,max_x,max_y;
    geometry_msgs::Point pt_in,pt_out;

    // transforming pts from pelvis to world frame
    pt_in.x = solar_pass_x_min;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    min_x = pt_out.x;
    pt_in.x = solar_pass_x_max;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    max_x = pt_out.x;
    pt_in.x =  0;
    pt_in.y = solar_pass_y_min;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    min_y = pt_out.y;
    pt_in.x =  0;
    pt_in.y = solar_pass_y_max;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    max_y = pt_out.y;


    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_x,max_x);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(min_y,max_y);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(solar_pass_z_min,solar_pass_z_max);
    pass_z.filter(*cloud);

}

void RoverBlocker::planeDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
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
/*
  double ak = pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2);
  double test = coefficients->values[0]/pow(ak,0.5);
  ROS_WARN_STREAM("cos angle :"<<test<<" acos "<<acos(test));
*/
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud);

//  ROS_INFO("Point cloud representing the planar component = %d", (int)cloud->points.size());

}




void RoverBlocker::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose){

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
    double xzslope = (pose.position.z - point2.z)/(pose.position.x - point2.x);

    ROS_INFO("slope is %f xz %f",slope,xzslope);


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

    double offset = 1.4;
    pose.position.x = pose.position.x - (offset*cos(theta));
    pose.position.y = pose.position.y - (offset*sin(theta));
    pose.position.z = 0.0;

//    ROS_INFO("Offset values to Footstep Planner are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);


    // if we need the orientation parallel to the walkway then use this.
    if(slope>0){//solar panel on the right
        theta+=1.5708;
    }
    else{ //solar panel on the left
        theta-=1.5708;
    }

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

