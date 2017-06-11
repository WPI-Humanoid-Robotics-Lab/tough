#include "val_task_common/panel_detection.h"

PanelDetector::PanelDetector(ros::NodeHandle &nh, DETECTOR_TYPE detector_type)
{
    current_state_ =  RobotStateInformer::getRobotStateInformer(nh);
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &PanelDetector::cloudCB, this);
    pcl_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/val_filter/filteredPointCloud", 1);

    vis_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    vis_plane_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_plane_vector", 1 );
    detection_tries_ = 0;
    //set the presets

    setPresetConfigs();

//    currentSettings_ = &preset_configs_[detector_type];
    currentDetector = detector_type;
    // resize plane model vector
    panel_plane_model_.resize(4);
}

PanelDetector::~PanelDetector()
{
    pcl_sub_.shutdown();

}

void PanelDetector::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val = detections_;
}

int PanelDetector::getDetectionTries() const
{
    return detection_tries_;
}

void PanelDetector::setDetectionTries(int detection_tries)
{
    detection_tries_ = detection_tries;
}

void PanelDetector::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input){

    if (input->data.empty())
        return;

    ++detection_tries_;
    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(*input, *cloud);

    passThroughFilter(cloud);
    if(cloud->empty())
        return;
    panelSegmentation(cloud);
    if(cloud->empty())
        return;
    segmentation(cloud);
    if(cloud->empty())
        return;

    geometry_msgs::Pose pose;
    if(getPosition(cloud, pose))
        detections_.push_back(pose);
    ros::Time endTime = ros::Time::now();

    //  std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;

    pcl_filtered_pub_.publish(output);
}

void PanelDetector::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(preset_configs_[currentDetector].x_min_limit,preset_configs_[currentDetector].x_max_limit);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(preset_configs_[currentDetector].y_min_limit, preset_configs_[currentDetector].y_max_limit);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(preset_configs_[currentDetector].z_min_limit,preset_configs_[currentDetector].z_max_limit);
    pass_z.filter(*cloud);
}

void PanelDetector::panelSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

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

    //update the plane model
    // if the upper plane id detected its c is almost 1. that plane is paraller to xy plane.
    // so for the safety we consider the detected plane is parallel to xy plane if c >=0.95
    // the lower plane c is around 0.87
    panel_plane_model_[0] = coefficients->values[0]; // a
    panel_plane_model_[1] = coefficients->values[1]; // b
    panel_plane_model_[2] = coefficients->values[2]; // c
    panel_plane_model_[3] = coefficients->values[3]; // d

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud);

    //  ROS_INFO("Point cloud representing the planar component = %d", (int)cloud->points.size());

}

bool PanelDetector::getPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, geometry_msgs::Pose& pose){

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

    double OFFSET = preset_configs_[currentDetector].OFFSET;

    // Not required anymore as the offsets are wrt Pelvis
//    if (preset_configs_[currentDetector].settingName == "HANDLE_PANEL_COARSE" || preset_configs_[currentDetector].settingName == "HANDLE_PANEL_FINE"){
//        if(pose.position.z > 0.80 && pose.position.z < 0.83)
//        if(panel_plane_model_[2]>=0.95)
//        {
//            ROS_INFO("Upper plane detected");
//            if(preset_configs_[currentDetector].settingName == "HANDLE_PANEL_COARSE")
//            {
//                OFFSET += 0.1;
//            }
//            else if (preset_configs_[currentDetector].settingName == "HANDLE_PANEL_FINE")
//            {
//                return false;
//            }
//        }
//        else if(pose.position.z > 0.75)
//        else if(panel_plane_model_[2]>0.8){
//            ROS_INFO("Lower Plane Detected");
//            //        OFFSET = 0.6;
//        }
//        else{
//            ROS_INFO("WTF");
//            return false;
//        }
//    }
    ROS_INFO("Centroid values are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);

    geometry_msgs::Point point1;
    point1.x = eigenVectors.col(2)[0] + pose.position.x;
    point1.y = eigenVectors.col(2)[1] + pose.position.y;
    point1.z = eigenVectors.col(2)[2] + pose.position.z;


    geometry_msgs::Point maxPoint1;
    geometry_msgs::Point minPoint1;

    maxPoint1.x = std::max(pose.position.x, point1.x);
    maxPoint1.y = std::max(pose.position.y, point1.y);
    maxPoint1.z = std::max(pose.position.z, point1.z);

    minPoint1.x = std::min(pose.position.x, point1.x);
    minPoint1.y = std::min(pose.position.y, point1.y);
    minPoint1.z = std::min(pose.position.z, point1.z);

    geometry_msgs::Point point2;
    point2.x = eigenVectors.col(0)[0] + pose.position.x;
    point2.y = eigenVectors.col(0)[1] + pose.position.y;
    point2.z = eigenVectors.col(0)[2] + pose.position.z;


    geometry_msgs::Point maxPoint2;
    geometry_msgs::Point minPoint2;

    maxPoint2.x = std::max(pose.position.x, point2.x);
    maxPoint2.y = std::max(pose.position.y, point2.y);
    maxPoint2.z = std::max(pose.position.z, point2.z);

    minPoint2.x = std::min(pose.position.x, point2.x);
    minPoint2.y = std::min(pose.position.y, point2.y);
    minPoint2.z = std::min(pose.position.z, point2.z);

    double slope = (pose.position.z - point2.z)/(pose.position.y - point2.y);

    float theta1 = 0;
    float cosTheta1 = 0;
    float sinTheta1 = 0;

    cosTheta1 = (maxPoint1.y - minPoint1.y)/(sqrt(pow((maxPoint1.x - minPoint1.x),2) + pow((maxPoint1.y - minPoint1.y),2) + pow((maxPoint1.z - minPoint1.z),2)));
    sinTheta1 = sqrt(1 - (pow(cosTheta1, 2)));

    if(slope > 0){

        theta1 = atan2(sinTheta1, cosTheta1) * -1.0;

    }
    else{

        theta1 = atan2(sinTheta1, cosTheta1);

    }

    ROS_INFO("The Orientation is given by := %0.2f", theta1);

    float theta2 = 0;
    float cosTheta2 = 0;
    float sinTheta2 = 0;

    cosTheta2 = -1*(maxPoint2.y - minPoint2.y)/(sqrt(pow((maxPoint2.x - minPoint2.x),2) + pow((maxPoint2.y - minPoint2.y),2) + pow((maxPoint2.z - minPoint2.z),2)));
    sinTheta2 = sqrt(1 - (pow(cosTheta2, 2)));
    theta2 = atan2(sinTheta2, cosTheta2);

    ROS_INFO("Magic angle is given by := %0.2f tanin %f slope %f", theta2, atan(slope), slope);
    //    double offset = 1.0;
    pose.position.x = pose.position.x - (OFFSET*cos(theta1));
    pose.position.y = pose.position.y - (OFFSET*sin(theta1));
    pose.position.z = 0.0;

    ROS_INFO("Offset values to Footstep Planner are X:= %0.2f, Y := %0.2f, Z := %0.2f", pose.position.x, pose.position.y, pose.position.z);

    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta1);

    pose.orientation = quaternion;

    visualization_msgs::Marker marker;

    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
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

    return true;
}

void PanelDetector::setPresetConfigs()
{
    geometry_msgs::Pose pelvisPose;
    current_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    //Handle panel coarse setting
    PanelSettings handle_panel_coarse;
    handle_panel_coarse.settingName = "HANDLE_PANEL_COARSE";

    handle_panel_coarse.x_min_limit = pelvisPose.position.x;
    handle_panel_coarse.x_max_limit = pelvisPose.position.x + 4.0f;

    handle_panel_coarse.y_min_limit = pelvisPose.position.y - 2.0f;
    handle_panel_coarse.y_max_limit = pelvisPose.position.x + 2.0f;

    handle_panel_coarse.z_min_limit = pelvisPose.position.z - 0.3f;
    handle_panel_coarse.z_max_limit = pelvisPose.position.z - 0.1f;

    handle_panel_coarse.OFFSET = 1.1f;

    preset_configs_[DETECTOR_TYPE::HANDLE_PANEL_COARSE] = handle_panel_coarse;

    //Handle panel fine setting
    PanelSettings handle_panel_fine = handle_panel_coarse;

    handle_panel_fine.settingName = "HANDLE_PANEL_FINE";
    handle_panel_fine.OFFSET = 0.7f;

    preset_configs_[DETECTOR_TYPE::HANDLE_PANEL_FINE] = handle_panel_fine;


}

void PanelDetector::segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

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

bool PanelDetector::getPanelPlaneModel(std::vector<float> &panelPlaneModel) const
{
    ROS_INFO("a : %0.4f, b : %0.4f, c : %0.4f, d: %.4f",panel_plane_model_[0], panel_plane_model_[1], panel_plane_model_[2], panel_plane_model_[3]);
    panelPlaneModel = panel_plane_model_;
    return panelPlaneModel.size() == 4;
}
