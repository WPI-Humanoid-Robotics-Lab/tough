#include <perception_common/walkway_generator.h>
#include <tough_common/val_common_names.h>
#include <perception_common/perception_common_names.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/time.h>

static const float GROUND_THRESHOLD      = 0.07f;
static const float FOOT_GROUND_THRESHOLD = 0.05f;

WalkwayGenerator::WalkwayGenerator(ros::NodeHandle &n){
    pointcloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("walkway",1);
    ROS_INFO("Subscribing to %s", PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC.c_str());
    pointcloudSub_ = nh_.subscribe(PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC, 1,  &WalkwayGenerator::generateWalkwayPoints, this);

    rd_ = RobotDescription::getRobotDescription(n);
}

WalkwayGenerator::~WalkwayGenerator(){
    pointcloudPub_.shutdown();
    pointcloudSub_.shutdown();
}

void WalkwayGenerator::generateWalkwayPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    if(cloud->empty())
        return;
    ROS_INFO("Recieved a pointcloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    float foot_height = getCurrentFootHeight();

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((foot_height - FOOT_GROUND_THRESHOLD - GROUND_THRESHOLD), foot_height - FOOT_GROUND_THRESHOLD + 0.1);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*out_cloud);

    // Downsample the cloud1
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> oct(0.02);
    oct.setInputCloud(out_cloud);
    oct.addPointsFromInputCloud();

    pcl::PointCloud<pcl::PointXYZ>::VectorType pts;
    oct.getVoxelCentroids(pts);

    out_cloud->clear();
    for (const auto &pt : pts) {
        out_cloud->push_back(pcl::PointXYZ());
        out_cloud->back().x = pt.x;
        out_cloud->back().y = pt.y;
        out_cloud->back().z = pt.z;
    }

    ROS_INFO("Computing Normals to each point");
    // Normals estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (out_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    ROS_INFO("Removing NaN Normals");
    std::vector<int> index;
    pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, index);

    auto inliers = boost::make_shared<pcl::PointIndices>();
    inliers->indices = index;
    inliers->header = cloud->header;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (out_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*out_cloud);

    if(out_cloud->empty())
    {
        ROS_ERROR("There are no normals in the point cloud!");
        return;
    }

    for(size_t i = 0; i < cloud_normals->points.size(); i++)
    {
        if(fabs(cloud_normals->points[i].normal_z) > SURFACE_NORMAL_THRESHOLD)
            temp_cloud2->points.push_back(out_cloud->points[i]);
    }

    if(temp_cloud2->empty())
    {
        ROS_ERROR("There are no points above the threshold...");
        return;
    }

    zAxisLimitFilter(temp_cloud2, cloud_filtered);

    if(getLargestCluster(cloud_filtered)) {
        cloud_filtered->header = cloud->header;

    pointcloudPub_.publish(cloud_filtered);

    }

}

void WalkwayGenerator::zAxisLimitFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud){

    float foot_height = getCurrentFootHeight();

    /* Filter out the pointcloud on z axis*/  
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (in_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits ((foot_height - FOOT_GROUND_THRESHOLD - GROUND_THRESHOLD), foot_height - FOOT_GROUND_THRESHOLD);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*out_cloud);
}

bool WalkwayGenerator::getLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    //create a kdtree for faster NN search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    int cloudSize = (int)cloud->points.size();

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);
    ec.setMinClusterSize ((int)(0.51 * cloudSize));    //get the cluster with atleast 10% of the total points. this can be 50%
    ec.setMaxClusterSize (cloudSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // If no cluster is found, return false and do not publish the message
    if(cluster_indices.empty())
        return false;

    auto inliers = boost::make_shared<pcl::PointIndices>();
    inliers->header.frame_id = "world";
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        inliers->indices = it->indices;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud);
    }
    return true;
}


double WalkwayGenerator::getCurrentFootHeight(void)
{
    double height_foot;
    tf::StampedTransform transformStamped;
    /// \todo use RobotState instead of tf
    try{
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, rd_->getLeftFootFrameName(), ros::Time(0),ros::Duration(3.0));
        tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF, rd_->getLeftFootFrameName(), ros::Time(0),transformStamped);
        height_foot = transformStamped.getOrigin().getZ();
        tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF, rd_->getRightFootFrameName(), ros::Time(0),transformStamped);
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        return 0.09; //this magic number is the default height of walkway
    }

    height_foot = height_foot > transformStamped.getOrigin().getZ() ? transformStamped.getOrigin().getZ() : height_foot;
    return height_foot;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "walkway_generator");
    ROS_INFO("Starting walkway filter node");
    ros::NodeHandle n;
    WalkwayGenerator obj(n);
    ros::spin();
    return 0;
}
