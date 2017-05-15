#include "val_task2/val_solar_panel_detector.h"
SolarPanelDetect::SolarPanelDetect(ros::NodeHandle nh, geometry_msgs::Pose rover_loc, bool isroverRight)
{
    pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &SolarPanelDetect::cloudCB, this);;
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_panel/cloud2", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "/val_solar_panel/visualization_marker", 1 );
    rover_loc_  =rover_loc;
    isroverRight_ = isroverRight;
//    ROS_INFO("rover loc const %f %f right: %d",rover_loc_.position.x,rover_loc_.position.y,(int)isroverRight_);
    setRoverTheta();
    setoffset();
//    ROS_INFO("in const");
}

SolarPanelDetect::~SolarPanelDetect()
{
    pcl_sub.shutdown();
    if(robot_state_ != nullptr)         delete robot_state_;
}
bool SolarPanelDetect::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val.clear();
    ret_val = detections_;
    return !ret_val.empty();

}

int SolarPanelDetect::getDetectionTries() const
{
    return detection_tries_;

}

void SolarPanelDetect::getoffset(float &minX, float &maxX,float &minY, float &maxY,float &minZ, float &maxZ)
{
    minX = min_x;
    maxX = max_x;
    minY = min_y;
    maxY = max_y;
    minZ = min_z;
    maxZ = max_z;
}

void SolarPanelDetect::setoffset(float minX, float maxX,float minY, float maxY,float minZ, float maxZ)
{
//    float slope =tan(rover_theta);
    ROS_INFO("setting offset");

    min_x = minX;
    max_x = maxX;
    min_y = minY;
    max_y = maxY;
    min_z = minZ;
    max_z = maxZ;
}


void SolarPanelDetect::setRoverTheta()
{
    // this func is for finding yaw and used for tranformation of the cloud
    tf::Quaternion quat(rover_loc_.orientation.x,rover_loc_.orientation.y,rover_loc_.orientation.z,rover_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);
    rover_theta = yaw;
    ROS_INFO("angle %.2f",rover_theta);
    if (isroverRight_)
    {
        rover_theta-=1.5708;
    }
    else
    {
        rover_theta+=1.5708;
    }

}

void SolarPanelDetect::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
{
    if (input->data.empty()){
        return;
    }
    ++detection_tries_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;
    pcl::fromROSMsg(*input, *cloud);


    transformCloud(cloud,true);
    PassThroughFilter(cloud);
    filter_solar_panel(cloud);
    transformCloud(cloud,false);

    geometry_msgs::Pose pose;
    getPosition(cloud,pose);
    detections_.push_back(pose);


//    visualizept(cloud->points[solar_panel_index[0]].x,cloud->points[solar_panel_index[0]].y,cloud->points[solar_panel_index[0]].z);



    ROS_INFO("pub size cloud %d",(int)cloud->points.size());

    pcl::toROSMsg(*cloud,output);
    output.header.frame_id="world";
    pcl_filtered_pub.publish(output);

}

void SolarPanelDetect::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,geometry_msgs::Pose &pose)
{
    // find centroid and apply pca
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pose.position.x = centroid(0);
    pose.position.y = centroid(1);
    pose.position.z = centroid(2);

    Eigen::Matrix3f covarianceMatrix;
    pcl::computeCovarianceMatrix(*cloud, centroid, covarianceMatrix);
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);

/*    ROS_INFO_STREAM("centroid x: "<<centroid(0)<<"centroid y: "<<centroid(1)<<"centroid z: "<<centroid(2));
    ROS_INFO_STREAM("Eigen value "<<eigenValues.col(0)<<"vec : "<<eigenVectors.col(0)[0]<<" "<<eigenVectors.col(0)[1]<<" "<<eigenVectors.col(0)[2]);
    ROS_INFO_STREAM("Eigen value "<<eigenValues.col(1)<<"vec : "<<eigenVectors.col(1)[0]<<" "<<eigenVectors.col(1)[1]<<" "<<eigenVectors.col(1)[2]);
    ROS_INFO_STREAM("Eigen value "<<eigenValues.col(2)<<"vec : "<<eigenVectors.col(2)[0]<<" "<<eigenVectors.col(2)[1]<<" "<<eigenVectors.col(2)[2]);
*/
    float cosTheta = eigenVectors.col(1)[0]/sqrt(pow(eigenVectors.col(1)[0],2)+pow(eigenVectors.col(1)[1],2));
    float sinTheta = eigenVectors.col(1)[1]/sqrt(pow(eigenVectors.col(1)[0],2)+pow(eigenVectors.col(1)[1],2));
    float theta = atan2(sinTheta, cosTheta);

//    ROS_INFO("theta %.2f",theta);
    // so that arrow is always inwards that is quadrant I and II
    if(isroverRight_)
    {
        if (theta >0)
        {
    //        ROS_INFO("hey");
            theta-=M_PI;
        }

    }
    else
    {
        if (theta <0)
        {
    //        ROS_INFO("hey");
            theta+=M_PI;
        }
    }

//    float slope = (cloud->points[0].x-cloud->points[1].x)/(cloud->points[0].y-cloud->points[1].y);

//    float theta1 = atan(slope);

//    theta = 0;
//    ROS_INFO("theta %.2f  theta1 %.2f",theta,theta1);

//    theta = theta1;

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
    pose.orientation = quaternion;

    visualizept(pose);
//    visualizept(centroid(0)+eigenVectors.col(1)[0],centroid(1)+eigenVectors.col(1)[1],centroid(2)+0*eigenVectors.col(1)[2]);
//    visualizept(centroid(0),centroid(1),centroid(2));

}


void SolarPanelDetect::filter_solar_panel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);

//    ROS_INFO("max z : %.2f",max_pt(2));  //max pt in z should alway between 0.9 and 1 (0.94/0.95 to be exact)

    // filtering around max z value
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
//    pass_z.setFilterLimits(max_pt(2)-0.08,max_pt(2));
    pass_z.setFilterLimits(max_pt(2)-0.06,max_pt(2));
    pass_z.filter(*cloud);

    //getting a pt in the handle
    for(int i = 0; i!=cloud->points.size();++i)
    {
        //ROS_WARN("pt z : %.2f",cloud->points[i].z);
        if(cloud->points[i].z>=(max_pt(2)-0.01))
        {
            //ROS_INFO("here z is %.2f",cloud->points[i].z);
            max_pt(0)=cloud->points[i].x;
            max_pt(1)=cloud->points[i].y;
            break;
        }
    }

    // clustering the points
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    int cloudSize = (int)cloud->points.size();
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize (10);  // magic number : 10   there should be atleast 10 point over the solar panel.
    ec.setMaxClusterSize (cloudSize-10); // same magic number
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    // we can find pts on the handle based on the euclidean distance
    ROS_INFO("number of clusters: %d",(int)cluster_indices.size());
    std::vector<float> euc_dist;
    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            float dist = pow(cloud->points[*pit].x-max_pt(0),2)+pow(cloud->points[*pit].y-max_pt(1),2)+pow(cloud->points[*pit].z-max_pt(2),2);
            euc_dist.push_back(dist);
            break;
        }
    }
    int index = std::min_element(euc_dist.begin(),euc_dist.end())-euc_dist.begin();
//    for(auto i = euc_dist.begin();i!=euc_dist.end();++i)
//    {
//        ROS_INFO("dist %.2f",*i);
//    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr solar_panel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (auto i = cluster_indices[index].indices.begin();i!=cluster_indices[index].indices.end();++i)
    {
        solar_panel_cloud->points.push_back(cloud->points[*i]);

    }
    cloud = solar_panel_cloud;
}

void SolarPanelDetect::visualizept(geometry_msgs::Pose pose)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Solar panel detector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(10);
    vis_pub.publish(marker);

}

void SolarPanelDetect::visualizept(float x,float y,float z)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Solar panel detector";
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
    vis_pub.publish(marker);


}

void SolarPanelDetect::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool isinverse)
{
//    ROS_INFO("trasnforming cloud");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << rover_loc_.position.x,rover_loc_.position.y,rover_loc_.position.z;
    transform.rotate (Eigen::AngleAxisf (rover_theta, Eigen::Vector3f::UnitZ()));
    if (isinverse)
    {
        transform = transform.inverse();
    }
    pcl::transformPointCloud (*cloud, *cloud, transform);
    ROS_INFO("trasnforming cloud");
}

void SolarPanelDetect::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


//    float min_x,min_y,max_x,max_y;
//    min_x = 0;
//    max_x = 1;
//    min_y = -1.5;
//    max_y =  1.5;

/*
    geometry_msgs::Point pt_in,pt_out;
    // transforming pts from pelvis to world frame
    pt_in.x = solar_pass_x_min;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    min_x = pt_out.x;    min_x = 0;
    max_x = 1;
    min_y = -1.5;
    max_y =  1.5;
    pt_in.x = solar_pass_x_max;min_x
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
