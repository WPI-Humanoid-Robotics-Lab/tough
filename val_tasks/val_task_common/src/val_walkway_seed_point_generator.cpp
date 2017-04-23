#include <val_task_common/val_walkway_seed_point_generator.h>
#include <val_common/val_common_names.h>
#include <perception_common/perception_common_names.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

static const float GROUND_THRESHOLD      = 0.07f;
static const float FOOT_GROUND_THRESHOLD = 0.05f;

WalkwaySeedPointGenerator::WalkwaySeedPointGenerator(ros::NodeHandle &n){
    pointcloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("testCloud",1);
    ROS_INFO("Subscribing to %s", PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC.c_str());
    pointcloudSub_ = nh_.subscribe(PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC, 1,  &WalkwaySeedPointGenerator::generateSeedPoints, this);
}

WalkwaySeedPointGenerator::~WalkwaySeedPointGenerator(){
    pointcloudPub_.shutdown();
    pointcloudSub_.shutdown();
}

void WalkwaySeedPointGenerator::generateSeedPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    if(cloud->empty())
        return;
    ROS_INFO("Recieved a pointcloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    zAxisLimitFilter(cloud, cloud_filtered);

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, index);

    if(getLargestCluster(cloud_filtered)) {
        cloud_filtered->header = cloud->header;

        float leafsize = 0.25;
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize (leafsize, leafsize, leafsize);
        grid.setInputCloud (cloud_filtered);
        grid.filter (*cloud_filtered);

        ROS_INFO("Cloud size after voxel grid filter : %d", (int)cloud_filtered->size());
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud_filtered);
        outrem.setRadiusSearch(1.5);
        outrem.setMinNeighborsInRadius (205);
        // apply filter
        outrem.filter (*cloud_filtered);

        ROS_INFO("Cloud size after outlier filter : %d", (int)cloud_filtered->size());

        pointcloudPub_.publish(cloud_filtered);
    }

}

void WalkwaySeedPointGenerator::zAxisLimitFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud){

    float foot_height = getCurrentFootHeight();

    /* Filter out the pointcloud on z axis*/
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());

    range_cond->addComparison(
                pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                    new pcl::FieldComparison<pcl::PointXYZ>(
                        "z", pcl::ComparisonOps::GT, foot_height - FOOT_GROUND_THRESHOLD - (GROUND_THRESHOLD))));

    range_cond->addComparison(
                pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                    new pcl::FieldComparison<pcl::PointXYZ>(
                        "z", pcl::ComparisonOps::LT, foot_height - FOOT_GROUND_THRESHOLD)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (in_cloud);
    condrem.setKeepOrganized(true);
    condrem.filter (*out_cloud);
}

bool WalkwaySeedPointGenerator::getLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    //create a kdtree for faster NN search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    int cloudSize = (int)cloud->points.size();

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);
    ec.setMinClusterSize ((int)(0.1*cloudSize));    //get the cluster with atleast 10% of the total points. this can be 50%
    ec.setMaxClusterSize (cloudSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // If no cluster is found, return false and do not publish the message
    if(cluster_indices.empty())
        return false;

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        *cloud = *cloud_cluster;
    }
    return true;
}


double WalkwaySeedPointGenerator::getCurrentFootHeight(void)
{
    double height_foot;
    tf::StampedTransform transformStamped;

    try{
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::L_FOOT_TF, ros::Time(0),ros::Duration(3.0));
        tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::L_FOOT_TF, ros::Time(0),transformStamped);
        height_foot = transformStamped.getOrigin().getZ();
        tf_listener_.lookupTransform( VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::R_FOOT_TF, ros::Time(0),transformStamped);
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        return 0.09; //this magic number is the default height of walkway
    }

    height_foot = height_foot > transformStamped.getOrigin().getZ() ? transformStamped.getOrigin().getZ() : height_foot;
    return height_foot;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "walkway_seed_generator");
    ROS_INFO("Starting walkway filter node");
    ros::NodeHandle n;
    WalkwaySeedPointGenerator obj(n);
    ros::spin();
    return 0;
}
