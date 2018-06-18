
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <pcl/range_image/range_image_planar.h>
#include <tough_common/tough_common_names.h>
#include <tough_filters/walkway_filter.h>
#include <tough_perception_common/perception_common_names.h>

WalkwayFilter::WalkwayFilter(ros::NodeHandle &n):nh_(n)
{
    pointcloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("walkway_filtered_points2",1, true);
    pointcloudSub_ = nh_.subscribe(PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC_FOR_OCTOMAP, 1,  &WalkwayFilter::generateMap, this);
    rd_ = RobotDescription::getRobotDescription(nh_);
}

WalkwayFilter::~WalkwayFilter()
{
    pointcloudPub_.shutdown();
    pointcloudSub_.shutdown();
}

void WalkwayFilter::generateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(cloud->empty())
        return;

    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
    outliers->header = cloud->header;

    float foot_height = getCurrentFootHeight();
    std::cout<<  foot_height << std::endl;

    for (size_t i = 0; i< cloud->size(); ++i){
        if( cloud->at(i).z < foot_height - FOOT_GROUND_THRESHOLD
                && cloud->at(i).z > foot_height - FOOT_GROUND_THRESHOLD - (GROUND_THRESHOLD)){
            outliers->indices.insert(outliers->indices.end(),i);
        }
    }
    subtractPointClouds(cloud,outliers);
    pointcloudPub_.publish(cloud);
}

double WalkwayFilter::getCurrentFootHeight(void)
{
    double height_foot;

    tf::StampedTransform transformStamped;
    tf_listener_.lookupTransform( TOUGH_COMMON_NAMES::WORLD_TF, rd_->getLeftFootFrameName(), ros::Time(0),transformStamped);
    height_foot = transformStamped.getOrigin().getZ();

    tf_listener_.lookupTransform( TOUGH_COMMON_NAMES::WORLD_TF, rd_->getRightFootFrameName(), ros::Time(0),transformStamped);

    height_foot = height_foot > transformStamped.getOrigin().getZ() ? transformStamped.getOrigin().getZ() : height_foot;

    return height_foot;
}

void WalkwayFilter::subtractPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices::Ptr outliers){
    pcl::ExtractIndices<pcl::PointXYZ> extract ;
    extract.setInputCloud(full_cloud);
    extract.setIndices(outliers);
    extract.setNegative (true);
    extract.filter (*full_cloud);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "walkway_filter");
    ROS_INFO("Starting walkway filter node");
    ros::NodeHandle n;
    WalkwayFilter m(n);
    ros::spin();
    return 0;
}
