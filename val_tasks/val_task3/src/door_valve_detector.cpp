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
    if(cloud->empty())
        return;
    filter_cloud(cloud);
    if(cloud->empty())
        return;
    transformCloud(cloud,1);
    fitting(cloud);
    if(cloud->empty())
        return;
    clustering(cloud);
    visualize();

    ros::Time endTime = ros::Time::now();

    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

    pcl_filtered_pub_.publish(output);

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

void DoorValvedetector::filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*cloud);

    // filtering around min x value
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_pt(0),min_pt(0)+0.05);
    pass_x.filter(*cloud);

}
void DoorValvedetector::fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // fitting a circle
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setRadiusLimits(0.2,0.3);
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
    extract.filter (*cloud);
    */
    for(int i=0;i<3;++i)
        circle_param.center.push_back(coefficients->values[i]);
    circle_param.radius = coefficients->values[3];
    for(int i=0;i<3;++i)
        circle_param.norm.push_back(coefficients->values[i+4]);
}

void DoorValvedetector::visualize()
{
    float cosTheta2 = circle_param.norm[0]/sqrt(pow(circle_param.norm[0],2)+pow(circle_param.norm[1],2));
    float sinTheta2 = circle_param.norm[1]/sqrt(pow(circle_param.norm[0],2)+pow(circle_param.norm[1],2));
    float theta2 = atan2(sinTheta2, cosTheta2);
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta2);

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

