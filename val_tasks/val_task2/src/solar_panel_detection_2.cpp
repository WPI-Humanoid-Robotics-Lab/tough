//
// Created by will on 5/31/17.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <val_task2/solar_panel_detection_2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>
#include <perception_common/perception_common_names.h>

// Trailer bounds relative to rover_pose, expressed as anything that can be passed to pcl::CropBox::setMin/setMax
#define TRAILER_BOUNDS_MAX {2.5f,  0.75f, 1.1f, 1}
#define TRAILER_BOUNDS_MIN {0.0f, -0.75f, 0.0f, 1}

solar_panel_detector_2::solar_panel_detector_2(ros::NodeHandle nh, const geometry_msgs::PoseStamped &rover_pose) :
        nh_(nh),
        rover_pose_(rover_pose),
        tf_listener_(nh),
        point_cloud_listener_(nh, "/leftFoot", "/left_camera_frame")
{
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("solar_panel_detection_debug_pose", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    blacklist_pub_ = nh_.advertise<PointCloud>("/block_map", 1);
    // Seems like the specific point type doesn't matter. I can publish PointXYZRGB to this topic and it works.
    points_pub_ = nh_.advertise<PointCloud>("solar_panel_detection_debug_points", 1);
    pcl_sub_ = nh.subscribe(PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC, 10, &solar_panel_detector_2::cloudCB, this);
    ROS_DEBUG("Solar panel detector 2 setup finished");
}

void solar_panel_detector_2::setRoverPose(const geometry_msgs::PoseStamped &rover_pose) {
    rover_pose_ = rover_pose;
}

geometry_msgs::PoseStamped solar_panel_detector_2::getRoverPose() const {
    return rover_pose_;
}

void solar_panel_detector_2::cloudCB(const PointCloud::ConstPtr &cloud_raw) {
    ROS_DEBUG_STREAM("Got point cloud of size " << cloud_raw->size());

    PointCloud::Ptr filtered_cloud = prefilterCloud(cloud_raw);
    if (filtered_cloud->empty()) {
        ROS_DEBUG("Abandoning solar panel detection attempt because the filtered cloud was empty");
        return;
    }
    ROS_DEBUG_STREAM("Filtered point cloud to size " << filtered_cloud->size() << " ("
                                                     << std::fixed << std::setprecision(1)
                                                     << (filtered_cloud->size() * 100.0 / cloud_raw->size())
                                                     << "%)");
    NormalCloud::Ptr filtered_normals = estimateNormals(filtered_cloud);

    pcl::PointIndices points_in_trailer;
    if (!findPointsInsideTrailer(filtered_cloud, filtered_normals, points_in_trailer)) {
        ROS_DEBUG("Abandoning solar panel detection attempt because the trailer could not be located");
        return;
    }

    points_pub_.publish(filtered_normals);
}

solar_panel_detector_2::PointCloud::Ptr solar_panel_detector_2::prefilterCloud(const PointCloud::ConstPtr &cloud_raw) const {
    // Get trailer pose as an Eigen
    geometry_msgs::PoseStamped rover_pose_cloud_frame;
    try {
        tf_listener_.transformPose(cloud_raw->header.frame_id, rover_pose_, rover_pose_cloud_frame);
    } catch (const tf::TransformException &e) {
        ROS_WARN_STREAM("Skipping detection attempt: could not transform rover pose into the cloud frame \n" << e.what());
        return boost::make_shared<PointCloud>();
    }

    pose_pub_.publish(rover_pose_cloud_frame);
    Eigen::Affine3d rover_pose;
    tf::poseMsgToEigen(rover_pose_cloud_frame.pose, rover_pose);

    // Crop the cloud to the trailer
    pcl::IndicesPtr points_in_trailer = boost::make_shared<std::vector<int>>();
    pcl::CropBox<Point> cropFilter;
    cropFilter.setInputCloud(cloud_raw);
    cropFilter.setMin(TRAILER_BOUNDS_MIN);
    cropFilter.setMax(TRAILER_BOUNDS_MAX);
    cropFilter.setTransform(rover_pose.inverse().cast<float>());

    cropFilter.filter(*points_in_trailer);

    // Transform the cropped point cloud into the trailer frame -- all further computations expect trailer to be
    // approximately axis-aligned
    auto points_in_trailer_aligned = boost::make_shared<PointCloud>();
    pcl::transformPointCloud(*cloud_raw, *points_in_trailer, *points_in_trailer_aligned, rover_pose.inverse());

    // Downsample the cloud -- using OctreePointCloudVoxelCentroid rather than VoxelGrid because VoxelGrid behaves badly
    // with large point clouds
    auto filtered_cloud = boost::make_shared<PointCloud>();
    filtered_cloud->header = cloud_raw->header;
    pcl::octree::OctreePointCloudVoxelCentroid<Point> oct(0.02);
    oct.setInputCloud(points_in_trailer_aligned);
    oct.addPointsFromInputCloud();

    PointCloud::VectorType pts;
    oct.getVoxelCentroids(pts);
    for (const auto &pt : pts) {
        filtered_cloud->push_back(Point());
        filtered_cloud->back().x = pt.x;
        filtered_cloud->back().y = pt.y;
        filtered_cloud->back().z = pt.z;
    }
    return filtered_cloud;
}

solar_panel_detector_2::NormalCloud::Ptr solar_panel_detector_2::estimateNormals(const PointCloud::ConstPtr &filtered_cloud) const {
    pcl::NormalEstimation<Point, Normal> ne;
    ne.setInputCloud(filtered_cloud);
    ne.setRadiusSearch(0.03);

    auto filtered_normals = boost::make_shared<NormalCloud>();
    filtered_normals->header = filtered_cloud->header;
    ne.compute(*filtered_normals);

    return filtered_normals;
}

bool solar_panel_detector_2::findPointsInsideTrailer(const PointCloud::ConstPtr &points,
                                                     const NormalCloud::Ptr &normals, // not ConstPtr because of PCL
                                                     pcl::PointIndices &pts_inside_trailer) const {
    // RGS to identify big segments. Probably better than RANSAC here. Probably better than RANSAC
    // in most other places I used RANSAC, too. Oh well.
    pcl::RegionGrowing<Point, Normal> rgs;
    rgs.setMinClusterSize(100); // probably can be much bigger than 100
    rgs.setNumberOfNeighbours(10);
    rgs.setInputCloud(points);
    rgs.setInputNormals(normals);

    auto clusters = boost::make_shared<std::vector<pcl::PointIndices>>();
    rgs.extract(*clusters);

    // Sort clusters so the largest cluster that fits each wall/floor description is encountered first
    std::sort(clusters->begin(), clusters->end(), [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
        return a.indices.size() < b.indices.size();
    });

//    publishClusters(points, *clusters);

    // Find the 3 walls and the floor
    pcl::ModelCoefficients front_model, left_model, right_model, floor_model;
    bool front_found, left_found, right_found, floor_found;

    pcl::SACSegmentationFromNormals<Point, Normal> sac;
    sac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setInputCloud(points);
    sac.setInputNormals(normals);
    // indices set inside the loop

    for (const pcl::PointIndices &cluster : *clusters) {
        pcl::PointIndices::ConstPtr cluster_ptr(clusters, &cluster); // aliasing pointer
        sac.setIndices(cluster_ptr);

        pcl::ModelCoefficients model;
        pcl::PointIndices inliers;
        sac.segment(inliers, model);

        // TODO Identify 3 walls and floor
    }
}


void solar_panel_detector_2::publishClusters(const PointCloud::ConstPtr &cloud, const std::vector<pcl::PointIndices> &clusters) const {
    pcl::PointCloud<pcl::PointXYZRGB> pub_cld;
    pub_cld.header = cloud->header;
    for (const pcl::PointIndices &cluster : clusters) {
        uint8_t r = std::rand() % 255;
        uint8_t g = std::rand() % 255;
        uint8_t b = std::rand() % 255;

        for (const int &idx : cluster.indices) {
            pcl::PointXYZRGB pt;
            pt.x = cloud->at(idx).x;
            pt.y = cloud->at(idx).y;
            pt.z = cloud->at(idx).z;
            pt.r = r;
            pt.g = g;
            pt.b = b;

            pub_cld.push_back(pt);
        }
    }
    points_pub_.publish(pub_cld);
}

Eigen::Vector3f solar_panel_detector_2::modelToVector(const pcl::ModelCoefficients &model) const {
    Eigen::Vector3f coef(model.values[0], model.values[1], model.values[2]);
    ROS_WARN_STREAM_COND(std::abs(1 - coef.norm()) > 0.01,
                         "Coefficients from RANSAC were not normalized (norm " << coef.norm() << ")");
    // normalize all directions to have positive y or, if y == 0, positive x
    if (coef.y() < 0 || (coef.y() == 0 && coef.x() < 0)) coef *= -1;
    return coef;
}

solar_panel_detector_2::~solar_panel_detector_2()
{
    pcl_sub_.shutdown();
}
