#include <val_task1/pcl_handle_detector.h>

pcl_handle_detector::pcl_handle_detector(ros::NodeHandle &nh, geometry_msgs::Pose panel_loc_)
{
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &pcl_handle_detector::cloudCB, this);
    pcl_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/val_task1/handle_detector/filteredPointCloud", 1);
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/visualization_marker_array", 1 );
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
    offset = 0.7;   //fine panel detection offset.
    panel_coarse_loc_ = panel_loc_;
    handle_loc_.clear();

}

bool pcl_handle_detector::getDetections(std::vector<geometry_msgs::Point> &detections)
{
    //index 0 is red, 1 is blue
    detections = handle_loc_;
    return handle_loc_.size() == 2;
}

pcl_handle_detector::~pcl_handle_detector()
{
    pcl_sub_.shutdown();
}


void pcl_handle_detector::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
{
    if (input->data.empty())
        return;
    handle_loc_.clear();
    ros::Time startTime = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;

    pcl::fromROSMsg(*input, *cloud);
    ROS_INFO("Receiving cloud size:  %d",(int)cloud->points.size());

    //  pcl::transformPointCloud(*cloud,*cloud,);
    //  transcloud(cloud);
    if (extractHandle(cloud))
        ROS_INFO("Success");
    else
        ROS_WARN("Failed");


    pcl::toROSMsg(*cloud, output);
    pcl_filtered_pub_.publish(output);
    ROS_INFO("publshing cloud size:  %d",(int)cloud->points.size());
    ros::Time endTime = ros::Time::now();
    std::cout << "Time Take for Calculating Position = " << endTime - startTime << std::endl;

}

bool pcl_handle_detector::extractHandle(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

    tf::Quaternion quat(panel_coarse_loc_.orientation.x,panel_coarse_loc_.orientation.y,panel_coarse_loc_.orientation.z,panel_coarse_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);
    double theta = yaw;


    ROS_INFO("theta : %.2f cos %.2f off %.2f",theta, cos(theta),offset);

    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;

    minPoint[0] = offset*0.5 * (1- fabs(cos(theta)));
    minPoint[1] = -0.45;
    minPoint[2] = 0.1;
    maxPoint[0] = minPoint[0]+0.9;
    maxPoint[1] = 0.45;
    maxPoint[2] = 0.4;

    ROS_INFO("Box Filter\n Min Pt x:%.2f y:%.2f z:%.2f\n Max Pt x:%.2f y:%.2f z:%.2f", minPoint[0],minPoint[1],minPoint[2],maxPoint[0],maxPoint[1],maxPoint[2]);

    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0]=panel_coarse_loc_.position.x;
    boxTranslatation[1]=panel_coarse_loc_.position.y;
    boxTranslatation[2]=0.6;   //Lower z location of the panel

    Eigen::Vector3f boxRotation;
    boxRotation[0]= 0;       // rotation around x-axis
    boxRotation[1]= 0;       // rotation around y-axis
    boxRotation[2]= theta;     // rotation around z-axis

    // apply box filter to get only the panel and knob
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

    if (cloud->empty()){
        return false;
    }

    // same as paneldetection ideally use the calculated values from there. (line 90-124)
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> cloudnew;
    cloudnew = *cloud;


    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.08);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (cloudnew);

    Eigen::Vector4f centroid;
    //  Calculating the Centroid of the Panel Point cloud
    pcl::compute3DCentroid(cloudnew, centroid);


    //  Using Priciple Component Analysis for computing the Orientation of the Panel
    Eigen::Matrix3f covarianceMatrix;
    pcl::computeCovarianceMatrix(cloudnew, centroid, covarianceMatrix);
    Eigen::Matrix3f eigenVectors;
    Eigen::Vector3f eigenValues;
    pcl::eigen33(covarianceMatrix, eigenVectors, eigenValues);
    tf::Vector3 unitz,normz,unitx,unity,normx,normy;

    unitz[0] = 0;unitz[0] = 0;unitz[2] = 1;
    unity[0] = 0;unity[0] = 1;unity[2] = 0;
    unitx[0] = 1;unitx[0] = 0;unitx[2] = 0;

    normz[0]=eigenVectors.col(2)[0];
    normz[1]=eigenVectors.col(2)[1];
    normz[2]=eigenVectors.col(2)[2];

    normy[0]=eigenVectors.col(1)[0];
    normy[1]=eigenVectors.col(1)[1];
    normy[2]=eigenVectors.col(1)[2];

    normx[0]=eigenVectors.col(0)[0];
    normx[1]=eigenVectors.col(0)[1];
    normx[2]=eigenVectors.col(0)[2];


    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << centroid(0), centroid(1), centroid(2);


    /*
    float angle;
    float slope = -1;
    if (slope >0)  // panel on the left of walkway
        angle = std::min(unity.angle(normz),unity.angle(-normz));
    else //panel  on the right of walkway
      angle = std::max(unity.angle(normz),unity.angle(-normz));

    float angle = std::min(unitz.angle(normx),unitz.angle(-normx));
    transform_2.rotate (Eigen::AngleAxisf (angle*0, Eigen::Vector3f::UnitX()));
    transform_2.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ()));
    ROS_INFO("Angles: %.2f %.2f and %.2f",angle,unity.angle(normz),unity.angle(-normz));
*/
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
    float ang = 0.523;  // 30 deg (trial and error) inclination of panel in y axis
    transform_3.rotate(Eigen::AngleAxisf (ang, Eigen::Vector3f::UnitY()));

    /* transforming the panel(point cloud) to the centroid of the panel
     * idea:  the inclination will be gone so a simple z-axis pass through
     * filter will remove the panel leaving only the two knobs.
     * */

    pcl::transformPointCloud (*cloud, *cloud, transform_2.inverse());
    pcl::transformPointCloud (*cloud, *cloud, transform_3);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.12,1);  // mostly is mean is at z=0.15
    pass_z.filter(*cloud);

    // pcl::transformPointCloud (*cloud, *cloud, transform_3.inverse());
    //  pcl::transformPointCloud (*cloud, *cloud, transform_2);

    // now clustering the knob to get the mean location of the two knobs
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    int cloudSize = (int)cloud->points.size();
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize (10);  // magic number : 10   there should be atleast 10 point over the knob.
    ec.setMaxClusterSize (cloudSize-10); // same magic number
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::vector<std::vector<float> > mean;

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        std::vector<float> loc_mean;
        float x=0.0,y=0.0,z=0.0;
        for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            x+=cloud->points[*pit].x;
            y+=cloud->points[*pit].y;
            z+=cloud->points[*pit].z;
        }
        loc_mean.push_back(x/it->indices.size());
        loc_mean.push_back(y/it->indices.size());
        loc_mean.push_back(z/it->indices.size());
        mean.push_back(loc_mean);
    }

    ROS_INFO("no. cluster %d",mean.size());

    // if number of cluster is more than two then there are things other than knob in the point cloud
    if (mean.size() !=2)
    {
        return false;
    }


    // a new cloud containing only the two mean location is created.
    ROS_INFO("mean 1 x %.2f y %.2f z %.2f",mean[0][0],mean[0][1],mean[0][2]);
    ROS_INFO("mean 2 x %.2f y %.2f z %.2f",mean[1][0],mean[1][1],mean[1][2]);

    pcl::PointCloud<pcl::PointXYZ> loc_cloud;
    pcl::PointXYZ data;

    if(mean[0][1]>0)
    {
        //location of red valve is mean[0]
        data.data[0] = mean[0][0];
        data.data[1] = mean[0][1];
        data.data[2] = mean[0][2];
        loc_cloud.push_back(data);
        data.data[0] = mean[1][0];
        data.data[1] = mean[1][1];
        data.data[2] = mean[1][2];
        loc_cloud.push_back(data);
    }
    else
    {
        //location of red valve is mean[1]
        data.data[0] = mean[1][0];
        data.data[1] = mean[1][1];
        data.data[2] = mean[1][2];
        loc_cloud.push_back(data);
        data.data[0] = mean[0][0];
        data.data[1] = mean[0][1];
        data.data[2] = mean[0][2];
        loc_cloud.push_back(data);
    }

    // transforming the new cloud (containing only 2 points) to its orginal location
    pcl::transformPointCloud (loc_cloud, loc_cloud, transform_3.inverse());
    pcl::transformPointCloud (loc_cloud, loc_cloud, transform_2);

    geometry_msgs::Point handle1, handle2;
    handle1.x = loc_cloud[0].data[0];
    handle1.y = loc_cloud[0].data[1];
    handle1.z = loc_cloud[0].data[2];

    handle2.x = loc_cloud[1].data[0];
    handle2.y = loc_cloud[1].data[1];
    handle2.z = loc_cloud[1].data[2];

    handle_loc_.push_back(handle1);
    handle_loc_.push_back(handle2);

    visualization_msgs::MarkerArray box_corners;
    visualization_msgs::Marker marker;

    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = loc_cloud[0].data[0];//mean[0][0];
    marker.pose.position.y = loc_cloud[0].data[1];
    marker.pose.position.z = loc_cloud[0].data[2];
    marker.pose.orientation.x = panel_coarse_loc_.orientation.x;
    marker.pose.orientation.y = panel_coarse_loc_.orientation.y;
    marker.pose.orientation.z = panel_coarse_loc_.orientation.z;
    marker.pose.orientation.w = panel_coarse_loc_.orientation.w;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(5);

    box_corners.markers.push_back(marker);

    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = loc_cloud[1].data[0];//mean[1][0];
    marker.pose.position.y = loc_cloud[1].data[1];//mean[1][1];
    marker.pose.position.z = loc_cloud[1].data[2];//mean[1][2];
    marker.pose.orientation.x = panel_coarse_loc_.orientation.x;
    marker.pose.orientation.y = panel_coarse_loc_.orientation.y;
    marker.pose.orientation.z = panel_coarse_loc_.orientation.z;
    marker.pose.orientation.w = panel_coarse_loc_.orientation.w;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(5);


    box_corners.markers.push_back(marker);

    vis_pub_.publish(box_corners);

    return true;

}
