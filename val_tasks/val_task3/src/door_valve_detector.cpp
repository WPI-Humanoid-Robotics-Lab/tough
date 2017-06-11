#include "val_task3/door_valve_detector.h"
#include "val_common/val_common_names.h"

DoorValvedetector::DoorValvedetector(ros::NodeHandle nh)
{
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &DoorValvedetector::cloudCB, this);
    pcl_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/val_door/cloud2", 1);
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/val_door/markers", 1 );
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
    setOffset();

}

DoorValvedetector::~DoorValvedetector()
{
    pcl_sub_.shutdown();
}

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

    float theta=0.0;

    boxfilter(cloud);
    transformCloud(cloud,0);
    /*
    PassThroughFilter(cloud);
    */
    if(cloud->empty())
    {
        ROS_INFO("empty after tansform cloud");
        return;
    }
//    filter_cloud(cloud);
    if(cloud->empty())
    {
        ROS_INFO("filter_cloud gives empty cloud");
        return;
    }
    transformCloud(cloud,1);
    panelSegmentation(cloud,theta);
    if(cloud->empty())
    {
        ROS_INFO("plane seg gives empty cloud");
        return;
    }
    return

    fitting(cloud,theta);
    if(cloud->empty())
        return;
    /*clustering(cloud);
*/

    geometry_msgs::Pose center_loc,center_loc_pelvis;

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(circle_param.theta);


    center_loc.position.x = circle_param.center[0];
    center_loc.position.y = circle_param.center[1];
    center_loc.position.z = circle_param.center[2];
    center_loc.orientation = quaternion;

    robot_state_->transformPose(center_loc,center_loc_pelvis,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);

    float pelvis_theta = tf::getYaw(center_loc_pelvis.orientation);


    float ang = (20*M_PI)/180;
    ROS_INFO(" pelvis theta %.4f  ang %.4f",pelvis_theta,ang);

    if(! (fabs(pelvis_theta) < ang) )
    {
        ROS_INFO(" pelvis theta %.4f",pelvis_theta);
        circle_param.theta+=M_PI;
    }

    quaternion = tf::createQuaternionMsgFromYaw(circle_param.theta);
    center_loc.orientation = quaternion;

    visualize();

    ros::Time endTime = ros::Time::now();

    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

    detections_.push_back(center_loc);
/*
    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

    pcl_filtered_pub_.publish(output);
*/
}

void DoorValvedetector::panelSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float &theta){

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
    ROS_INFO("a : %0.4f, b : %0.4f, c : %0.4f, d: %.4f",coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);

    pcl::ExtractIndices<pcl::PointXYZ> extract1;
    extract1.setInputCloud (cloud);
    extract1.setIndices (inliers);
    extract1.setNegative (true);
    extract1.filter (*cloud);

    // find centroid and apply pca
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plane_cloud, centroid);
    Eigen::Matrix3f covarianceMatrix;
    pcl::computeCovarianceMatrix(*plane_cloud, centroid, covarianceMatrix);
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);

//    visualizept(centroid(0)+eigenVectors.col(0)[0],centroid(1)+eigenVectors.col(0)[1],centroid(2));

    float cosTheta = eigenVectors.col(0)[0]/sqrt(pow(eigenVectors.col(0)[0],2)+pow(eigenVectors.col(0)[1],2));
    float sinTheta = eigenVectors.col(0)[1]/sqrt(pow(eigenVectors.col(0)[0],2)+pow(eigenVectors.col(0)[1],2));
    theta = atan2(sinTheta, cosTheta);



    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    ROS_INFO("plne");
    pcl_filtered_pub_.publish(output);

}


void DoorValvedetector::boxfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
       geometry_msgs::Pose pelvisPose;
       robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
       Eigen::Vector4f minPoint;
       Eigen::Vector4f maxPoint;
       minPoint[0]= 0;
       minPoint[1]=-0.6;
       minPoint[2]= -0.2; //pelvis height is approx 0.1

       maxPoint[0]=2;
       maxPoint[1]=+0.6;
       maxPoint[2]=0.8;

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
       box_filter.setInputCloud(cloud);
       box_filter.setMin(minPoint);
       box_filter.setMax(maxPoint);
       box_filter.setTranslation(boxTranslatation);
       box_filter.setRotation(boxRotation);
       box_filter.setNegative(false);
       box_filter.filter(*cloud);
       ROS_INFO("box filter");
}

void DoorValvedetector::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    ROS_INFO("clustering");
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    int cloudSize = (int)cloud->points.size();
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize (50);  // magic number : 50   there should be atleast 10 point over the solar panel.
    ec.setMaxClusterSize (cloudSize-50); // same magic number
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    ROS_INFO("number of clusters: %d",(int)cluster_indices.size());

    if(!(int)cluster_indices.size())
        return;
    // if cluster is zero then dont process further

    std::vector<float> euc_dist;
    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            float dist = pow(cloud->points[*pit].x-circle_param.center[0],2)+pow(cloud->points[*pit].y-circle_param.center[1],2)+pow(cloud->points[*pit].z-circle_param.center[2],2);
            euc_dist.push_back(dist);
            break;
        }
    }
    int index = std::min_element(euc_dist.begin(),euc_dist.end())-euc_dist.begin();
//    for(auto i = euc_dist.begin();i!=euc_dist.end();++i)
//    {
//        ROS_INFO("dist %.2f",*i);
//    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr door_valve_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (auto i = cluster_indices[index].indices.begin();i!=cluster_indices[index].indices.end();++i)
    {
        door_valve_cloud ->points.push_back(cloud->points[*i]);

    }
    cloud = door_valve_cloud;

}

bool DoorValvedetector::getDetections(std::vector<geometry_msgs::Pose> &out)
{
   out = detections_;
   return !detections_.empty();
}

void DoorValvedetector::filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

    ROS_INFO("filter cloud");
    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);

    ROS_INFO("min x %.2f max x%.2f",min_pt(0),max_pt(0));

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*cloud);


    // filtering around min x value
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_pt(0),max_pt(0)-0.15);
    pass_x.filter(*cloud);

}
void DoorValvedetector::fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float &theta)
{

    // voxel filter to prevent improper fitting of circle
    pcl::VoxelGrid<pcl::PointXYZ> sor;
     sor.setInputCloud (cloud);
     sor.setLeafSize (0.01f, 0.01f, 0.01f);
     sor.filter (*cloud);

     Eigen::Vector4f min_pt,max_pt;
     pcl::getMinMax3D(*cloud,min_pt,max_pt);


    // fitting a circle
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setRadiusLimits(0.18,0.3);
    //seg.setAxis(ax);
    seg.setDistanceThreshold (0.008);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    ROS_INFO("x : %0.4f, y : %0.4f, z : %0.4f, r: %.4f ",coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

/*
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud);*/
/*

    if(coefficients->values[0] < min_pt(0) || coefficients->values[0] > max_pt(0))
    {
        coefficients->values[0] = (min_pt(0)+max_pt(0))/2;
    }


    if(coefficients->values[1] < min_pt(1) || coefficients->values[1] > max_pt(1))
    {
        coefficients->values[1] = (min_pt(1)+max_pt(1))/2;
    }


    if(coefficients->values[2] < min_pt(2) || coefficients->values[2] > max_pt(2))
    {
        coefficients->values[2] = (min_pt(2)+max_pt(2))/2;
    }
*/

/*
    coefficients->values[0] = (min_pt(0)+max_pt(0))/2;
    coefficients->values[1] = (min_pt(1)+max_pt(1))/2;
    coefficients->values[2] = (min_pt(2)+max_pt(2))/2;
    coefficients->values[3] = max_pt(2) - coefficients->values[2];
*/

    circle_param.center.clear();

    for(int i=0;i<3;++i)
        circle_param.center.push_back(coefficients->values[i]);
    circle_param.radius = coefficients->values[3];
    circle_param.theta = theta;

}

void DoorValvedetector::visualize()
{
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(circle_param.theta);

    visualization_msgs::MarkerArray mk_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Normal Vector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = circle_param.center[0];
    marker.pose.position.y = circle_param.center[1];
    marker.pose.position.z = circle_param.center[2];
    marker.pose.orientation = quaternion;
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
    marker.ns = "Center";
    marker.pose.position.x = circle_param.center[0];
    marker.pose.position.y = circle_param.center[1];
    marker.pose.position.z = circle_param.center[2];
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    mk_array.markers.push_back(marker);

    marker.ns = "Sample Radius pt";
    marker.pose.position.x = circle_param.center[0];
    marker.pose.position.y = circle_param.center[1];
    marker.pose.position.z = circle_param.center[2]+circle_param.radius;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mk_array.markers.push_back(marker);

    vis_pub_.publish(mk_array);
}

void DoorValvedetector::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    float minx,miny,minz,maxx,maxy,maxz;
    geometry_msgs::Point pt_in,pt_out;
    ROS_INFO("pts %.2f %.2f %.2f %.2f %.2f %.2f",min_x,max_x,min_y,max_y,min_z,max_z);

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


void DoorValvedetector::visualizept(float x,float y,float z)
{
    visualization_msgs::MarkerArray mk_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Door valve detector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(10);
    mk_array.markers.push_back(marker);

    vis_pub_.publish(mk_array);

}
