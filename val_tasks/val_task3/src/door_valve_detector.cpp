#include "val_task3/door_valve_detector.h"
#include "val_common/val_common_names.h"

DoorValvedetector::DoorValvedetector(ros::NodeHandle nh)
{
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &DoorValvedetector::cloudCB, this);
    pcl_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/val_door/cloud2", 1);
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/val_door/Position", 1 );

    robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
    setOffset();

}

DoorValvedetector::~DoorValvedetector()
{
    pcl_sub_.shutdown();
}
/*
void DoorValvedetector::setTransform()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF,now,ros::Duration(10.0));
        listener.lookupTransform(VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF,ros::Time(0),transform);
        ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
    }
    catch{

    }
}*/

void DoorValvedetector::setOffset(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
    min_x = minX;
    max_x = maxX;
    min_y = minY;
    max_y = maxY;
    min_z = minZ;
    max_z = maxZ;
}

void DoorValvedetector::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input){

    if (input->data.empty()){
        return;
    }

    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(*input, *cloud);


/*  // use this when walking over the steps this will helps us in reducing the pts before transforming
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(2,10);
    pass_z.filter(*cloud);

*/
    transformCloud(cloud,0);
    PassThroughFilter(cloud);
//    transformCloud(cloud,1);

    ros::Time endTime = ros::Time::now();

    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

    pcl_filtered_pub_.publish(output);

}


void DoorValvedetector::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    float minx,miny,minz,maxx,maxy,maxz;
    geometry_msgs::Point pt_in,pt_out;
    ROS_INFO("pts %.2f %.2f %.2f %.2f %.2f %.2f",min_x,max_x,min_y,max_y,min_z,max_z);

/*


    // transforming pts from pelvis to world frame
    pt_in.x = min_x;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    minx = pt_out.x;
    pt_in.x = max_x;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    maxx = pt_out.x;
    pt_in.x =  0;
    pt_in.y = min_y;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    miny = pt_out.y;
    pt_in.x =  0;
    pt_in.y = max_y;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    maxy = pt_out.y;
    pt_in.x =  0;
    pt_in.y =  0;
    pt_in.z = min_z;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    minz = pt_out.z;
    pt_in.x =  0;
    pt_in.y =  0;
    pt_in.z = max_z;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    maxz = pt_out.z;
*/
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
    pass_z.setFilterLimits(min_z,max_z);
    pass_z.filter(*cloud);

}

void DoorValvedetector::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool isinverse)
{
//    ROS_INFO("trasnforming cloud");
    geometry_msgs::Pose pelvis,p2;
    pelvis.position.x = 0;
    pelvis.position.y = 0;
    pelvis.position.z = 0;
    pelvis.orientation.x = 0;
    pelvis.orientation.y = 0;
    pelvis.orientation.z = 0;
    pelvis.orientation.w = 1;
    robot_state_->transformPose(pelvis,p2,VAL_COMMON_NAMES::PELVIS_TF);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << p2.position.x,p2.position.y,0.0;
    transform.rotate (Eigen::AngleAxisf (-tf::getYaw(p2.orientation), Eigen::Vector3f::UnitZ()));
    if (isinverse)
    {
        transform = transform.inverse();
    }
    pcl::transformPointCloud (*cloud, *cloud, transform);
    ROS_INFO("trasnforming cloud");
}


/*
void RoverDetector::getPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr& lowerBoxCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& upperBoxCloud, std::vector<geometry_msgs::Pose>& detectedPoses){

    detectedPoses.clear();
    detectedPoses.resize(3); // 3 detections - coarse, fine, finer

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

    //    double offset = finePose_ ? 6.45 : 6.45;
    double xOffset[3] = {6.45, 6.35, 6.05};
    double thetaOffset[3] = {M_PI_2, M_PI_4, 0.0};
    double yOffset[3] = {-0.6 , -0.2, -0.2};
    for (int i =0; i < 3; ++i){
        detectedPoses[i].position.x = upperBoxPosition.x - (xOffset[i]*cos(theta));
        detectedPoses[i].position.y = upperBoxPosition.y - (xOffset[i]*sin(theta));
        detectedPoses[i].position.z = 0.0;

        //    ROS_INFO("Offset values to Footstep Planner are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);
        geometry_msgs::Quaternion quaternion;
        ROS_INFO("slopeyz %.2f, theta %.2f",yzSlope,theta);

        float angle;

        if(yzSlope>0)  {//rover on the left
            //            angle = finePose_ ? theta : theta-1.5708;
            angle = theta - thetaOffset[i];
            quaternion = tf::createQuaternionMsgFromYaw(angle);
            roverSide_ = ROVER_SIDE::LEFT;
        }
        else {
            //rover on the right
            //            angle = finePose_ ? theta : theta+1.5708;
            angle = theta + thetaOffset[i];
            quaternion = tf::createQuaternionMsgFromYaw(angle);
            roverSide_ = ROVER_SIDE::RIGHT;
        }



        //    float yOffset = finePose_ ? -0.2 : -0.6;
        detectedPoses[i].position.x = detectedPoses[i].position.x + yOffset[i] * cos(angle);
        detectedPoses[i].position.y = detectedPoses[i].position.y + yOffset[i] * sin(angle);

        detectedPoses[i].orientation = quaternion;
    }

    visualization_msgs::MarkerArray markerArray;
    int id=0;
    for(auto pose : detectedPoses){
            visualization_msgs::Marker marker;

            marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
            marker.header.stamp = ros::Time();
            marker.ns = "Rover Position";
            marker.id = id++;
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
            marker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(marker);
    }
    vis_pub_.publish(markerArray);

}

bool RoverDetector::getDetections(std::vector<std::vector<geometry_msgs::Pose> > &ret_val)
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
*/
