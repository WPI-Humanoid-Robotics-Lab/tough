#include "val_task2/val_solar_panel_detector.h"

void SolarPanelDetect::invertYaw(geometry_msgs::Pose &pose)
{
    // for valkyrie orientation :
    // pointing inwards : if panel is behind valkyrie: theta
    // pointing outwards : if panel is in front of valkyrie: theta-M_PI
    tfScalar r, p, y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    tf::Matrix3x3 rot(q);
    rot.getRPY(r, p, y);
    y = y-M_PI;
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
    pose.orientation = quaternion;
}

SolarPanelDetect::SolarPanelDetect(ros::NodeHandle nh, geometry_msgs::Point button_loc)
{
    pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &SolarPanelDetect::cloudCB, this);;
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_panel/cloud2", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 1 );

    robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);
    ROS_INFO("pose : x %.2f y %.2f z %.2f yaw %.2f ",current_pelvis_pose.position.x,current_pelvis_pose.position.y,current_pelvis_pose.position.z,tf::getYaw(current_pelvis_pose.orientation));

//    optimal_dist = 0.16;
    button_loc_ = button_loc;

//    ROS_INFO("rover loc const %f %f right: %d",rover_loc_.position.x,rover_loc_.position.y,(int)isroverRight_);
//    setRoverTheta();
    setoffset();
    ROS_INFO("optimaldistance %.2f",optimal_dist);
//    ROS_INFO("in const");
}

SolarPanelDetect::SolarPanelDetect(ros::NodeHandle nh)
{
    pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &SolarPanelDetect::cloudCB, this);;
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_panel/cloud2", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 1 );
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,current_pelvis_pose);
    ROS_INFO("pose : x %.2f y %.2f z %.2f yaw %.2f ",current_pelvis_pose.position.x,current_pelvis_pose.position.y,current_pelvis_pose.position.z,tf::getYaw(current_pelvis_pose.orientation));

//    optimal_dist = 0.16;

    button_loc_.x = 0;
    button_loc_.y = 0;
    button_loc_.z = 0;

//    ROS_INFO("rover loc const %f %f right: %d",rover_loc_.position.x,rover_loc_.position.y,(int)isroverRight_);
//    setRoverTheta();
    setoffset();
//    ROS_INFO("in const");
}


SolarPanelDetect::~SolarPanelDetect()
{
    pcl_sub.shutdown();
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

void SolarPanelDetect::getoffset(float &minX, float &maxX,float &minY, float &maxY,float &minZ, float &maxZ, float &optimalDist, float &pitchDeg)
{
    minX = min_x;
    maxX = max_x;
    minY = min_y;
    maxY = max_y;
    minZ = min_z;
    maxZ = max_z;
    optimalDist = optimal_dist;
    pitchDeg = (pitch*180)/M_PI;
}

void SolarPanelDetect::setoffset(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float optimalDist,float pitchDeg)
{
//    float slope =tan(rover_theta);
    ROS_INFO("setting offset and other parameters");

    min_x = minX;
    max_x = maxX;
    min_y = minY;
    max_y = maxY;
    min_z = minZ;
    max_z = maxZ;
    optimal_dist = optimalDist;
    pitch = (pitchDeg*M_PI)/180;
}

/*
void SolarPanelDetect::setRoverTheta()
{
    // this func is for finding yaw and used for tranformation of the cloud
    rover_theta = rover_loc_.theta;
    ROS_INFO("angle %.2f",rover_theta);
    /*
     * If the rover orientation given is parallel to walkway
    if (isroverRight_)
    {
        rover_theta-=1.5708;
    }
    else
    {
        rover_theta+=1.5708;
    }

}*/

void SolarPanelDetect::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
{
    if (input->data.empty()){
        return;
    }
    ++detection_tries_;

    geometry_msgs::Pose pose;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;
    pcl::fromROSMsg(*input, *cloud);

    boxfilter(cloud);

    if(cloud->empty())
    {
        ROS_INFO("box fiter empty");
        return;
    }

    filter_solar_panel(cloud,pose);

    if(cloud->empty())
    {
        ROS_WARN("Positon not found, empty cloud");
        return;
    }

    getOrientation(cloud,pose);
    detections_.push_back(pose);

    /*transformCloud(cloud,true);
    PassThroughFilter(cloud);
    if(cloud->empty())
        return;
    filter_solar_panel(cloud);
    if(cloud->empty())
        return;
    transformCloud(cloud,false);

    geometry_msgs::Pose pose;
    getPosition(cloud,pose);
    detections_.push_back(pose);

*/
//    visualizept(cloud->points[solar_panel_index[0]].x,cloud->points[solar_panel_index[0]].y,cloud->points[solar_panel_index[0]].z);



    pcl::toROSMsg(*cloud,output);
    output.header.frame_id=VAL_COMMON_NAMES::WORLD_TF;
    pcl_filtered_pub.publish(output);

}

void SolarPanelDetect::boxfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

       geometry_msgs::Pose pelvisPose;
       robot_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
       Eigen::Vector4f minPoint;
       Eigen::Vector4f maxPoint;
       minPoint[0]= 0 ;
       minPoint[1]=-0.3;
       minPoint[2]= 0.48;  //0.46 also works, very few points lie on the rover plane

       maxPoint[0]=1;
       maxPoint[1]=+0.3;
       maxPoint[2]=1;

       Eigen::Vector3f boxTranslatation;
       boxTranslatation[0]=pelvisPose.position.x;
       boxTranslatation[1]=pelvisPose.position.y;
       boxTranslatation[2]=0*pelvisPose.position.z;
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


void SolarPanelDetect::PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients ){

//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.008);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    ROS_INFO("a : %0.4f, b : %0.4f, c : %0.4f, d: %.4f",coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    /*panel_plane_model_[0] = coefficients->values[0]; // a
    panel_plane_model_[1] = coefficients->values[1]; // b
    panel_plane_model_[2] = coefficients->values[2]; // c
    panel_plane_model_[3] = coefficients->values[3]; // d
    */
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud);

    ROS_INFO("Point cloud representing the planar component = %d", (int)cloud->points.size());


}


void SolarPanelDetect::getOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, geometry_msgs::Pose &pose)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);


    ROS_INFO("max z : %.2f",max_pt(2));

    // filtering around max z value
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
//    pass_z.setFilterLimits(max_pt(2)-0.08,max_pt(2));
    pass_z.setFilterLimits(0,max_pt(2)-0.15);
    pass_z.filter(*cloud);

    ROS_INFO("cld size %d",(int)cloud->points.size());

    if (cloud->points.empty()){
        ROS_WARN("z Pass thru gives empty cloud in get orientation ");
        return;
    }

    PlaneSegmentation(cloud,inliers,coefficients);

    if (cloud->points.empty()){
        ROS_WARN("No plane found");
        return;
    }


    // PCA
//    transformCloud(cloud,tf::getYaw(current_pelvis_pose.orientation)+(M_PI/2),true);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    Eigen::Matrix3f covarianceMatrix;
    pcl::computeCovarianceMatrix(*cloud, centroid, covarianceMatrix);
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);

//    transformCloud(cloud,tf::getYaw(current_pelvis_pose.orientation)+(M_PI/2),false);

    /*float area = getArea(cloud,inliers,coefficients);
    ROS_INFO("Area of the plane %.4f",area);
    ROS_INFO("Normalised Area of the plane %.4f",(area*100)/cloud->points.size());
    */


    float cosTheta = eigenVectors.col(0)[0]/sqrt(pow(eigenVectors.col(0)[0],2)+pow(eigenVectors.col(0)[1],2));
    float sinTheta = eigenVectors.col(0)[1]/sqrt(pow(eigenVectors.col(0)[0],2)+pow(eigenVectors.col(0)[1],2));
    float theta = atan2(sinTheta, cosTheta);

    transformCloud(cloud,theta,true);
    pcl::getMinMax3D(*cloud,min_pt,max_pt);
    ROS_INFO("range %.4f",fabs(max_pt(1)-min_pt(1)));
    transformCloud(cloud,theta,false);

//    if the length is  less than 0.3 then detected plane is the smaller side of the panel
    if (fabs(max_pt(1)-min_pt(1)) < 0.3)
    {
        ROS_INFO("Smaller plane detected");
        cosTheta = eigenVectors.col(1)[0]/sqrt(pow(eigenVectors.col(1)[0],2)+pow(eigenVectors.col(1)[1],2));
        sinTheta = eigenVectors.col(1)[1]/sqrt(pow(eigenVectors.col(1)[0],2)+pow(eigenVectors.col(1)[1],2));
        theta = atan2(sinTheta, cosTheta);
    }

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,pitch,theta);
    pose.orientation = quaternion;

    geometry_msgs::Pose pose_pelvis;
//    robot_state_->transformPose(pose,pose_pelvis,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
    pose_pelvis = pose;


    // so that arrow is always inwards
    if (!(fabs(tf::getYaw(pose_pelvis.orientation)) <= (M_PI/2)))
    {
        ROS_INFO("orientation was outward");
        theta+=M_PI;
    }
    ROS_INFO("Theta %.4f",theta);

    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,pitch,theta);
    pose.orientation = quaternion;
    visualizept(pose);

}

void SolarPanelDetect::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,geometry_msgs::Pose &pose)
{
    // find centroid and apply pca
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
//    pose.position.x = centroid(0);
//    pose.position.y = centroid(1);
//    pose.position.z = centroid(2);

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

    // for viz
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,pitch,theta);
    pose.orientation = quaternion;
//    visualizept(pose);

    /*
    tfScalar r, p, y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(quaternion, q);
    tf::Matrix3x3 rot(q);
    rot.getRPY(r, p, y);
    ROS_INFO("Roll %.2f, Pitch %.2f, Yaw %.2f", (float)r, (float)p, (float)y);
//    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
    */


    //NOTE: if there is problem with the position try uncommenting
    // line with transformCloud at below two places

//    transformCloud(cloud,theta,1);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);

    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);

    ROS_INFO("x %.4f y %.4f",max_pt(0)-min_pt(0),max_pt(1)-min_pt(1));

//    float length = std::max(max_pt(0)-min_pt(0),max_pt(1)-min_pt(1));
    /*pcl::PointCloud<pcl::PointXYZ> pos_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pos_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

//    pcl::PointCloud<pcl::PointXYZ> pos;//((max_pt(0)+min_pt(0))/2,(max_pt(1)+min_pt(1))/2,(max_pt(2)+min_pt(2))/2);
    pos_cloud.width=1;
    pos_cloud.height=1;
    pos_cloud.points.resize(1);

    size_t i=0;
    pos_cloud.points[i].x = (max_pt(0)+min_pt(0))/2;
    pos_cloud.points[i].y = (max_pt(1)+min_pt(1))/2;
    pos_cloud.points[i].z = (max_pt(2)+min_pt(2))/2;

    pos_cloud_ptr->points.push_back(pos_cloud.points[0]);

//    transformCloud(pos_cloud_ptr,theta,0);

    pose.position.x = pos_cloud_ptr->points[0].x;
    pose.position.y = pos_cloud_ptr->points[0].y;
    pose.position.z = pos_cloud_ptr->points[0].z;
*/

    pose.position.x = (max_pt(0)+min_pt(0))/2;
    pose.position.y = (max_pt(1)+min_pt(1))/2;
    pose.position.z = (max_pt(2)+min_pt(2))/2;


    ROS_WARN("Is button loc avail??");

    if(button_loc_.z != 0)
    {
        ROS_INFO("Yes,button loc is avail");
        float dist = sqrt(pow(pose.position.x-button_loc_.x,2)+pow(pose.position.y-button_loc_.y,2));
        ROS_INFO("Distance is %.2f",dist);
        if (dist > 0.19 || dist < 0.13)
        {
            ROS_INFO("Moving position towards/away from button");
            float lambda = optimal_dist/dist;
            pose.position.x = button_loc_.x - lambda*(button_loc_.x-pose.position.x);
            pose.position.y = button_loc_.y - lambda*(button_loc_.y-pose.position.y);

        }
    }

/*
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud,output);
    output.header.frame_id=VAL_COMMON_NAMES::WORLD_TF;
    pcl_filtered_pub.publish(output);

    visualizept(pose);*/
}


// NOTE: cloud is not by reference
void SolarPanelDetect::filter_solar_panel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::Pose &pose)
{
    Eigen::Vector4f min_pt,max_pt;
    pcl::getMinMax3D(*cloud,min_pt,max_pt);

//    ROS_INFO("max z : %.2f",max_pt(2));  //max pt in z should alway between 0.9 and 1 (0.94/0.95 to be exact)

    pcl::PointCloud<pcl::PointXYZ>::Ptr pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // filtering around max z value
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
//    pass_z.setFilterLimits(max_pt(2)-0.08,max_pt(2));
    pass_z.setFilterLimits(max_pt(2)-0.06,max_pt(2));
    pass_z.filter(*pos_cloud);

    if(pos_cloud->empty())
    {
        ROS_WARN("Panel handle Pass through filter Empty inside filter solar panel func");
        return;
    }

    ROS_INFO("cloud size before clustering %d",(int)pos_cloud->points.size());
    getPosition(pos_cloud,pose);


    /*

    //getting a pt in the handle
    for(int i = 0; i<=cloud->points.size();++i)
    {
        //ROS_WARN("pt z : %.2f",cloud->points[i].z);
//        this condition might be the reason for choosing the cluster wrongly (occasionally)
//        if(cloud->points[i].z>=(max_pt(2)-0.01))
        if(cloud->points[i].z>=(max_pt(2)))
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
    ec.setMaxClusterSize (cloudSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    // we can find pts on the handle based on the euclidean distance
//    ROS_INFO("number of clusters: %d",(int)cluster_indices.size());

    if(!(int)cluster_indices.size())
        return;
    // if cluster is zero then dont process further

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
//    getPosition(solar_panel_cloud,pose);
//    cloud = solar_panel_cloud;

*/

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

void SolarPanelDetect::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float theta, bool isinverse)
{
//    ROS_INFO("trasnforming cloud");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << current_pelvis_pose.position.x,current_pelvis_pose.position.y,0.0;
//    transform.rotate (Eigen::AngleAxisf (tf::getYaw(current_pelvis_pose.orientation)+(M_PI/2), Eigen::Vector3f::UnitZ()));
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    if (isinverse)
    {
        transform = transform.inverse();
    }
    pcl::transformPointCloud (*cloud, *cloud, transform);
    ROS_INFO("trasnforming cloud");
}

void SolarPanelDetect::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


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
