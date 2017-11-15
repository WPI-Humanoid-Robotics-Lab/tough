#include "val_task2/val_solar_detection.h"
#include <pcl_ros/point_cloud.h>

#define solar_pass_x_min -2.0
#define solar_pass_x_max  10.0
#define solar_pass_y_min -10.0
#define solar_pass_y_max  10.0
#define solar_pass_z_min  1.38
#define solar_pass_z_max  1.91

/* x axis : -2 to 10
 * y axis : -10 to 10
 * z axis : 1.38 to 1.91
 * */


SolarArrayDetector::SolarArrayDetector(ros::NodeHandle nh, geometry_msgs::Pose2D rover_loc, bool isroverRight)
{
  pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &SolarArrayDetector::cloudCB, this);
  pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_plane/cloud2", 1);
  vis_pub_array = nh.advertise<visualization_msgs::MarkerArray>( "/visualization_marker_array", 1 );
  rover_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/block_map",1);
    dbg_points_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/solar_detection_debug_pts", 1);

  rover_loc_ = rover_loc;
  robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh);
  detection_tries_ = 0;
  detections_.clear();
  isroverRight_ = isroverRight;
}
SolarArrayDetector::~SolarArrayDetector()
{
      pcl_sub.shutdown();
}

bool SolarArrayDetector::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val.clear();
    ret_val = detections_;
    return !ret_val.empty();

}
int SolarArrayDetector::getDetectionTries() const
{
    return detection_tries_;

}


void SolarArrayDetector::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
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
    dbg_points_pub_.publish(cloud);
    pcl::PCLHeader cld_header = cloud->header;
//    ROS_INFO("removing rover");
    roverremove(cloud);
    cloud->header = cld_header;

    PassThroughFilter(cloud);
//    ROS_INFO("pub %d",(int)cloud->points.size());

    planeDetection(cloud);
    getPosition(cloud,location);
    detections_.push_back(location);

    pcl::toROSMsg(*cloud,output);
    output.header.frame_id="world";
//    pcl_filtered_pub.publish(output);

    ros::Time endTime = ros::Time::now();
//    std::cout << "Number of detections               = " << detections_.size() << std::endl;
//    std::cout << "Number of tries                    = " << detection_tries_ << std::endl;
//    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

}

void SolarArrayDetector::roverremove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    /*tf::Quaternion quat(rover_loc_.orientation.x,rover_loc_.orientation.y,rover_loc_.orientation.z,rover_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);
    */
    double theta;
    theta = rover_loc_.theta;
/*
    if (isroverRight_)
    {
        theta = rover_loc_.theta-1.5708; // rover right of walkway
    }
    else
    {
        theta = rover_loc_.theta+1.5708;   //left of walkway
    }
*/

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    minPoint[0]=0;
    minPoint[1]=-2;
    minPoint[2]=0;

    maxPoint[0]=9;
    maxPoint[1]=+2;
    maxPoint[2]=3;
    Eigen::Vector3f boxTranslatation;
         boxTranslatation[0]=rover_loc_.x;
         boxTranslatation[1]=rover_loc_.y;
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

    // Widen cloud in +/- y to make sure val doesn't walk into the side of it
    Eigen::Affine3f rover_shift_l, rover_shift_r;
    const auto rover_align = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());

    rover_shift_l.linear() = Eigen::Matrix3f::Identity();
    rover_shift_l.translation() = rover_align * Eigen::Vector3f(-0.2, -0.4, 0);
    rover_shift_r.linear() = Eigen::Matrix3f::Identity();
    rover_shift_r.translation() = rover_align * Eigen::Vector3f(-0.2, 0.4, 0);

    pcl::PointCloud<pcl::PointXYZ> rover_shifted_l, rover_shifted_r;
    pcl::transformPointCloud(rover_cloud, rover_shifted_l, rover_shift_l);
    pcl::transformPointCloud(rover_cloud, rover_shifted_r, rover_shift_r);

    rover_shifted_l += rover_shifted_r;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(rover_shifted_l, rover_output);
    //ROS_INFO("rover cloud size %d",(int)rover_cloud.points.size());
    output.header.frame_id="world";
    rover_cloud_pub.publish(rover_output);


    box_filter.setNegative(true);
    box_filter.filter(*cloud);
//    box_filter.filter(indices);


}

void SolarArrayDetector::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


    float min_x,min_y,max_x,max_y;
    geometry_msgs::Point pt_in,pt_out;

    // transforming pts from pelvis to world frame
    pt_in.x = solar_pass_x_min;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,rd_->getPelvisFrame());
    min_x = pt_out.x;
    pt_in.x = solar_pass_x_max;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,rd_->getPelvisFrame());
    max_x = pt_out.x;
    pt_in.x =  0;
    pt_in.y = solar_pass_y_min;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,rd_->getPelvisFrame());
    min_y = pt_out.y;
    pt_in.x =  0;
    pt_in.y = solar_pass_y_max;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,rd_->getPelvisFrame());
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

void SolarArrayDetector::planeDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
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

}




void SolarArrayDetector::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose){

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

//    ROS_INFO("Centroid values are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);
/*
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


    //double slope = (pose.position.z - point2.z)/(pose.position.y - point2.y);
    //double xzslope = (pose.position.z - point2.z)/(pose.position.x - point2.x);

    //ROS_INFO("slope is %f xz %f",slope,xzslope);


    float theta = 0;
    float cosTheta = 0;
    float sinTheta = 0;



    cosTheta = (maxPoint.x - minPoint.x)/(sqrt(pow((maxPoint.x - minPoint.x),2) + pow((maxPoint.y - minPoint.y),2) + pow((maxPoint.z - minPoint.z),2)));
    sinTheta = (maxPoint.y - minPoint.y)/(sqrt(pow((maxPoint.x - minPoint.x),2) + pow((maxPoint.y - minPoint.y),2) + pow((maxPoint.z - minPoint.z),2)));//sqrt(1 - (pow(cosTheta, 2)));


    double yzSlope = (maxPoint.z - minPoint.z)/(maxPoint.y - minPoint.y);

    bool noSlope = (fabs((maxPoint.y - minPoint.y) < 0.01));

    double xySlope = 0.0;

    if(!noSlope){
        xySlope = (maxPoint.x - minPoint.x)/(maxPoint.y - minPoint.y);
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
    }*/

    float cosTheta1 = eigenVectors.col(2)[0]/sqrt(pow(eigenVectors.col(2)[0],2)+pow(eigenVectors.col(2)[1],2));
    float sinTheta1 = eigenVectors.col(2)[1]/sqrt(pow(eigenVectors.col(2)[0],2)+pow(eigenVectors.col(2)[1],2));
    float theta1 = atan2(sinTheta1, cosTheta1);  // should be the final yaw for parallel to solar array



    float cosTheta2 = eigenVectors.col(0)[0]/sqrt(pow(eigenVectors.col(0)[0],2)+pow(eigenVectors.col(0)[1],2));
    float sinTheta2 = eigenVectors.col(0)[1]/sqrt(pow(eigenVectors.col(0)[0],2)+pow(eigenVectors.col(0)[1],2));
    float theta2 = atan2(sinTheta2, cosTheta2);

    // eigenVectors.col(0)[2] < 0 // pointing downwards or towards the walkway // perpendicular to solar array

    if (eigenVectors.col(0)[2] < 0)
    {
        theta2+=M_PI;

    }

//    ROS_INFO("green marker t1 %.2f",theta1);
//    ROS_INFO("blue marker t2 %.2f",theta2);

    // coarse location
    float off = -2.2;
    pose.position.x = centroid(0) + off*cos(theta2);
    pose.position.y = centroid(1) + off*sin(theta2);
    pose.position.z = 0;//centroid(2);

//    ROS_INFO_STREAM("Testing:\n eigen value: "<<eigenValues.col(0)<<"\nVec: "
//                    <<eigenVectors.col(0)<<"eigen value: "<<eigenValues.col(1)
//                    <<"\nVec: "<<eigenVectors.col(1)<<"eigen value: "<<eigenValues.col(2)
//                    <<"\nVec: "<<eigenVectors.col(2));

    /*if(slope > 0){

        theta = atan2(sinTheta, cosTheta) * -1.0;
    }
    else{

        theta = atan2(sinTheta, cosTheta);
    }

    ROS_INFO("The Orientation is given by := %0.2f", theta);
    */
    /*
    double offset = 2.0;
    pose.position.x = pose.position.x - (offset*cos(theta));
    pose.position.y = pose.position.y - (offset*sin(theta));
    pose.position.z = 0;//pose.position.z;

    ROS_INFO("slopeyz %.2f, theta %.2f",yzSlope,theta);
    */
//    ROS_INFO("Offset values to Footstep Planner are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);


    // if we need the orientation parallel to the walkway then use this.
    /*if(slope>0){//solar panel on the right
        theta+=1.5708;
    }
    else{ //solar panel on the left
        theta-=1.5708;
    }*/

    /*
    float angle;
//geometry_msgs::Quaternion quaternion;
      if(yzSlope>0)  {//rover on the left
          angle = theta-1.5708;
//          quaternion = tf::createQuaternionMsgFromYaw(theta-1.5708);
//          isRoverOnRight_ = std::make_shared<bool>(false);
      }
      else {
          //rover on the right
          angle = theta+1.5708;
//          quaternion = tf::createQuaternionMsgFromYaw(theta+1.5708);
//          isRoverOnRight_ = std::make_shared<bool>(true);
      }

      float distance_offset = -1;
      pose.position.x = pose.position.x + distance_offset * cos(angle);
      pose.position.y = pose.position.y + distance_offset * sin(angle); */


      //uncomment after this
//    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(angle);
      //testing
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta2);

    pose.orientation = quaternion;

    visualization_msgs::MarkerArray mk_array;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Center of the Panel";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    /*marker.pose.position.x = centroid(0);
    marker.pose.position.y = centroid(1);
    marker.pose.position.z = centroid(2);
    marker.pose.orientation.x = pose.orientation.x;
    marker.pose.orientation.y = pose.orientation.y;
    marker.pose.orientation.z = pose.orientation.z;
    marker.pose.orientation.w = pose.orientation.w;*/
    marker.pose = pose;
    marker.scale.x = 0.6;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(5);

    mk_array.markers.push_back(marker);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    /*
    marker.ns = "maxPoint";
    marker.pose.position= maxPoint;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    mk_array.markers.push_back(marker);
    marker.ns = "minPoint";
    marker.pose.position= minPoint;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mk_array.markers.push_back(marker);
    marker.ns = "Point1";
    marker.pose.position= point1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    mk_array.markers.push_back(marker);
    marker.ns = "Point2";
    marker.pose.position= point2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    mk_array.markers.push_back(marker);
    marker.ns = "centroid";
    marker.pose.position.x = centroid(0);
    marker.pose.position.y = centroid(1);
    marker.pose.position.z = centroid(2);
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    mk_array.markers.push_back(marker);
*/
    marker.ns = "vec 1";
    marker.pose.position.x = centroid(0)+eigenVectors.col(0)[0];
    marker.pose.position.y = centroid(1)+eigenVectors.col(0)[1];
    marker.pose.position.z = centroid(2)+eigenVectors.col(0)[2];
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    mk_array.markers.push_back(marker);
    marker.ns = "vec 2";
    marker.pose.position.x = centroid(0)+eigenVectors.col(1)[0];
    marker.pose.position.y = centroid(1)+eigenVectors.col(1)[1];
    marker.pose.position.z = centroid(2)+eigenVectors.col(1)[2];
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    mk_array.markers.push_back(marker);
    marker.ns = "vec 3";
    marker.pose.position.x = centroid(0)+eigenVectors.col(2)[0];
    marker.pose.position.y = centroid(1)+eigenVectors.col(2)[1];
    marker.pose.position.z = centroid(2)+eigenVectors.col(2)[2];
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mk_array.markers.push_back(marker);

    vis_pub_array.publish(mk_array);


}
