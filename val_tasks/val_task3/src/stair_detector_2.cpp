//
// Created by will on 5/31/17.
//

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

// PCL
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>

// Local
#include <perception_common/perception_common_names.h>
#include <val_task3/stair_detector_2.h>
#include <pcl/segmentation/region_growing.h>

#define RANDOM_STEP_ARROW_COLOR false

#define N_STAIR_ANGLE_BUCKETS 16
// These values were derived by inspecting objects in Gazebo
#define STEP_DEPTH 0.2389f
#define STEP_HEIGHT 0.2031f

stair_detector_2::stair_detector_2(ros::NodeHandle nh) :
        nh_(nh),
        utils_(nh),
        tf_listener_(nh),
        point_cloud_listener_(nh, "/leftFoot", "/left_camera_frame")
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("stairs", 1);
    blacklist_pub_ = nh_.advertise<stair_detector_2::PointCloud>("/block_map", 1);
    points_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZL>>("stair_detection_debug_points", 1);
    pcl_sub_ = nh.subscribe(PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC, 10, &stair_detector_2::cloudCB, this);
    ROS_DEBUG("stair detector setup finished");
}

void stair_detector_2::cloudCB(const PointCloud::ConstPtr &cloud_raw) {
    ROS_DEBUG_STREAM("Got point cloud of size " << cloud_raw->size());

    PointCloud::Ptr filtered_cloud = prefilterCloud(cloud_raw);
    ROS_DEBUG_STREAM("Filtered point cloud to size " << filtered_cloud->size() << " (" << std::setprecision(1)
                                                     << (filtered_cloud->size() * 100.0 / cloud_raw->size()) << "%)");
    NormalCloud::Ptr filtered_normals = estimateNormals(filtered_cloud);
    ROS_DEBUG_STREAM("Normal estimation complete");
//    publishNormals(*filtered_cloud, *filtered_normals);

    std::vector<pcl::PointIndices> clusters;
    Eigen::Vector3f stairs_dir;
    if (!estimateStairs(filtered_cloud, filtered_normals)) return;
}

float stair_detector_2::estimateStairPose(const PointCloud::ConstPtr &filtered_cloud,
                                          const std::vector<pcl::PointIndices> &step_clusters,
                                          const Eigen::Vector3f &stairs_dir, const Eigen::Vector3f &robot_pos,
                                          Eigen::Affine3f &stairs_pose) const {
    float stairs_angle = std::atan2(stairs_dir.y(), stairs_dir.x());

    Eigen::Affine3f vis_angle(Eigen::AngleAxisf(stairs_angle, Eigen::Vector3f::UnitZ()));

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker test_angle_m;
    pcl_conversions::fromPCL(filtered_cloud->header, test_angle_m.header);
    test_angle_m.ns = "test_angle" ;
    test_angle_m.id = 0;
    test_angle_m.type = visualization_msgs::Marker::ARROW;
    tf::poseEigenToMsg(vis_angle.cast<double>(), test_angle_m.pose);
    test_angle_m.scale.x = 1;
    test_angle_m.scale.y = 0.05;
    test_angle_m.scale.z = 0.05;
    test_angle_m.color.r = 1;
    test_angle_m.color.g = 0;
    test_angle_m.color.b = 1;
    test_angle_m.color.a = 1;
    ma.markers.push_back(test_angle_m);

    ROS_DEBUG_STREAM("Testing angle " << stairs_angle);

    std::vector<pcl::PointIndices> stair_clusters;
    auto step_shape = boost::make_shared<PointCloud>();

    step_shape->header = filtered_cloud->header;

    for (const auto &cluster : step_clusters) {
        // Get the centroid of each cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*filtered_cloud, cluster.indices, centroid);
        // Categorize which step this is based on the centroid height
        float step_height = (centroid.z() + 0.1f) / STEP_HEIGHT;

        size_t step = static_cast<size_t>(step_height);
        if (step_height - step < 0.1 || step_height - step > 0.9) {
            // Discard clusters whose centroids are too close to the boundary between steps to confidently categorize
            continue;
        }


        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*filtered_cloud, cluster.indices, min, max);
        if (max.z() - min.z() > STEP_HEIGHT + 0.05) {
            // Discard clusters which are too tall to represent a single step
            continue;
        }

        // Make sure that step exists in the cluster list
        if (stair_clusters.size() <= step) stair_clusters.resize(step + 1);
        // Add this cluster's points to the given step (note that indices are guaranteed to be unique)
        std::copy(cluster.indices.begin(), cluster.indices.end(), back_inserter(stair_clusters.at(step).indices));

        step_shape->reserve(step_shape->size() + cluster.indices.size());
        Eigen::Affine3f step_tf(Eigen::Translation3f(-STEP_DEPTH * step, 0, -STEP_HEIGHT * step)
                                * Eigen::AngleAxisf(stairs_angle, Eigen::Vector3f::UnitZ()).inverse());
        for (const auto &idx : cluster.indices) {
            Point pt_tf = pcl::transformPoint(filtered_cloud->at(idx), step_tf);
            step_shape->push_back(pt_tf);
        }
    }

//    publishClusters(filtered_cloud, step_clusters);

    const long n_steps_found = count_if(stair_clusters.begin(), stair_clusters.end(),
                                            [](const pcl::PointIndices &c) { return c.indices.size() > 0; });

    if (n_steps_found < 5) {
        ROS_DEBUG_STREAM("Candidate step pose abandoned because only " << n_steps_found << " steps were found");
        //  ros::Duration(2).sleep();
        marker_pub_.publish(ma);
        return std::numeric_limits<float>::infinity();
    }

    // all steps should now be overlaid on top of each other, entirely between -0.1 < z < 0. (within some error)
    pcl::PassThrough<Point> pt;
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-0.12f, 0.12f);
    pt.setInputCloud(step_shape);
    auto step_clipped_indices = boost::make_shared<pcl::PointIndices>();
    pt.filter(step_clipped_indices->indices);

    // find the largest euclidean segment -- it should be very dense and by far the largest
    pcl::EuclideanClusterExtraction<Point> cluster;
    cluster.setClusterTolerance(0.02);
    cluster.setMinClusterSize(1000);
    cluster.setInputCloud(step_shape);
    cluster.setIndices(step_clipped_indices);

    std::vector<pcl::PointIndices> shape_clusters;
    cluster.extract(shape_clusters);

//    publishClusters(step_shape, shape_clusters);

    if (shape_clusters.empty()) {
        ROS_DEBUG_STREAM("Candidate step pose abandoned because the steps were not at the expected positions for this angle");
        //  ros::Duration(2).sleep();
        marker_pub_.publish(ma);
        return std::numeric_limits<float>::infinity();
    }

    auto stair_cluster_iter = std::max_element(
            shape_clusters.begin(), shape_clusters.end(),
            [](const pcl::PointIndices &a, const pcl::PointIndices &b) { return a.indices.size() < b.indices.size(); }
    );

    if (stair_cluster_iter->indices.size() < step_clipped_indices->indices.size() / 2) {
        ROS_DEBUG_STREAM("Candidate step pose abandoned because the steps were not at the expected positions for this angle");
        //  ros::Duration(2).sleep();
        marker_pub_.publish(ma);
        return std::numeric_limits<float>::infinity();
    }

    auto stair_cluster = boost::make_shared<pcl::PointIndices>();
    stair_cluster->indices = std::move(stair_cluster_iter->indices);

    ROS_DEBUG_STREAM("Biggest cluster had " << stair_cluster->indices.size() << " (" << shape_clusters.size() << " clusters)");

    // PCA to get centroid and improved estimate of the angle
    pcl::PCA<Point> pca;
    pca.setInputCloud(step_shape);
    pca.setIndices(stair_cluster);

    // Translation identified by bounding box center, not mean
    stairs_pose.translation() = Eigen::Vector3f::Zero();

    // Rotation of PCA
    Eigen::Matrix3f rot_mat = pca.getEigenVectors();

    // Reorder principal components into a valid rotation matrix
    stairs_pose.linear().col(0) = (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
    stairs_pose.linear().col(2) = rot_mat.col(1);
    stairs_pose.linear().col(1) = rot_mat.col(0);

    // Transform the whole cloud one last time in order to get the stair center and variance
    auto aligned_step_shape = boost::make_shared<PointCloud>();
    pcl::transformPointCloud(*step_shape, stair_cluster->indices, *aligned_step_shape, stairs_pose.inverse());
    points_pub_.publish(aligned_step_shape);
    Eigen::Vector4f obb_min, obb_max;
    pcl::getMinMax3D(*aligned_step_shape, obb_min, obb_max);
    stairs_pose.translation() = stairs_pose * ((obb_max.head<3>() + obb_min.head<3>()) / 2);
    // Score is sum of sizes in each dimension, the smaller the better
    float score = (obb_max.head<3>() - obb_min.head<3>()).sum();

    // Apply the inverse to the rotation from before
    stairs_pose = Eigen::AngleAxisf(stairs_angle, Eigen::Vector3f::UnitZ()) * stairs_pose;

    // Make sure the smallest principal component is in the xy axis (test whether the transform rotates the x axis away
    // from the xy plane)
    if ((stairs_pose.linear() * Eigen::Vector3f::UnitX()).z() > 0.1) {
        ROS_DEBUG_STREAM("Candidate step pose abandoned because the resulting angle was not in the xy plane");
        //  ros::Duration(2).sleep();
        marker_pub_.publish(ma);
        return std::numeric_limits<float>::infinity();
    }

    // Make sure z points up and not down
    if ((stairs_pose.linear() * Eigen::Vector3f::UnitZ()).z() < 0) {
        ROS_DEBUG("Flipping Z to point up");
        stairs_pose.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    }

    // Robot pose in stairs frame should have x < 0
    if ((stairs_pose.inverse() * robot_pos).x() > 0) {
        ROS_DEBUG("Flipping X to point away from robot");
        stairs_pose.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
    }

    visualization_msgs::Marker candidate_pose_m;
    pcl_conversions::fromPCL(filtered_cloud->header, candidate_pose_m.header);
    candidate_pose_m.ns = "candidate_pose" ;
    candidate_pose_m.id = 0;
    candidate_pose_m.type = visualization_msgs::Marker::ARROW;
    tf::poseEigenToMsg(stairs_pose.cast<double>(), candidate_pose_m.pose);
    candidate_pose_m.scale.x = 1;
    candidate_pose_m.scale.y = 0.05;
    candidate_pose_m.scale.z = 0.05;
    candidate_pose_m.color.r = 1;
    candidate_pose_m.color.g = 1;
    candidate_pose_m.color.b = 0;
    candidate_pose_m.color.a = 1;
    ma.markers.push_back(candidate_pose_m);

    ROS_DEBUG_STREAM("Got a potential pose with score " << score);
    //  ros::Duration(2).sleep();
    marker_pub_.publish(ma);
    return score;
}

stair_detector_2::PointCloud::Ptr stair_detector_2::prefilterCloud(const PointCloud::ConstPtr &cloud_raw) const {
    // Crop the cloud to the height of the staircase
    pcl::IndicesPtr points_in_room = boost::make_shared<std::vector<int>>();
    pcl::CropBox<Point> cropFilter;
    cropFilter.setInputCloud(cloud_raw);
    cropFilter.setMin({0, -8.f, 0.02f, 1});
    cropFilter.setMax({8.f, 8.f, 2.f, 1});

    cropFilter.filter(*points_in_room);

    // Downsample the cloud1
    auto filtered_cloud = boost::make_shared<PointCloud>();
    filtered_cloud->header = cloud_raw->header;
    pcl::octree::OctreePointCloudVoxelCentroid<Point> oct(0.01);
    oct.setInputCloud(cloud_raw, points_in_room);
    oct.addPointsFromInputCloud();

    PointCloud::VectorType pts;
    oct.getVoxelCentroids(pts);
    int ctr = 0;
    for (const auto &pt : pts) {
        filtered_cloud->push_back(Point());
        filtered_cloud->back().x = pt.x;
        filtered_cloud->back().y = pt.y;
        filtered_cloud->back().z = pt.z;

        ROS_WARN_STREAM_COND(filtered_cloud->back().getVector3fMap().norm() < 0.01, "Point " << ctr << " was "
                             << filtered_cloud->back().getVector3fMap().transpose());

        ctr++;
    }
    return filtered_cloud;
}

stair_detector_2::NormalCloud::Ptr stair_detector_2::estimateNormals(const PointCloud::ConstPtr &filtered_cloud) const {
    pcl::NormalEstimation<Point, Normal> ne;
    ne.setInputCloud(filtered_cloud);
    ne.setRadiusSearch(0.04);

    auto filtered_normals = boost::make_shared<NormalCloud>();
    filtered_normals->header = filtered_cloud->header;
    ne.compute(*filtered_normals);

    return filtered_normals;
}

bool stair_detector_2::estimateStairs(const PointCloud::ConstPtr &filtered_cloud,
                                      const NormalCloud::Ptr &filtered_normals) {
    // Use RGS to identify the continuous planar regions
    pcl::RegionGrowing<Point, Normal> rgs;
    rgs.setMinClusterSize(250); // probably can be much bigger than 100
    rgs.setNumberOfNeighbours(30);
    rgs.setCurvatureThreshold(0.15);
    rgs.setSmoothnessThreshold(40 * M_PI / 180);
    rgs.setSmoothModeFlag(false);
    rgs.setInputCloud(filtered_cloud);
    rgs.setInputNormals(filtered_normals);

    auto regions = boost::make_shared<std::vector<pcl::PointIndices>>();
    rgs.extract(*regions);

    ROS_DEBUG_STREAM("Region growing segmentation found " << regions->size() << " regions");
//    publishClusters(filtered_cloud, *regions);

    // Use RANSAC to get direction of each regions and filter out the non-vertical ones
    pcl::SACSegmentationFromNormals<Point, Normal> nsac;
    nsac.setOptimizeCoefficients(true);
    nsac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    nsac.setMethodType(pcl::SAC_RANSAC);
    nsac.setDistanceThreshold(0.06);
    nsac.setInputCloud(filtered_cloud);
    nsac.setInputNormals(filtered_normals);

    std::vector<pcl::PointIndices> clusters;
    std::vector<pcl::ModelCoefficients> models;

    for (const pcl::PointIndices &region : *regions) {
        pcl::PointIndices::ConstPtr region_ptr(regions, &region); // Aliasing constructor

        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;

        // Segment the largest planar component from the remaining cloud
        nsac.setIndices(region_ptr);
        nsac.segment(inliers, coefficients);
        Eigen::Vector3f coef = modelToVector(coefficients);

        if (std::abs(coef.z()) < 0.1 && inliers.indices.size() > 250) {
            models.push_back(std::move(coefficients));
            clusters.push_back(std::move(inliers));
        }
    }

    if (clusters.size() < 6) {
        ROS_DEBUG_STREAM("Abandoning stair detection attempt, only " << clusters.size() << " clusters found");
        return false;
    }

    // Use quickly-implemented hough transform to identify the angle of the stairs
    std::array<int, N_STAIR_ANGLE_BUCKETS> stair_angle_buckets{}; // the {} causes everything to be initialized to 0

    for (std::size_t i = 0; i < clusters.size(); i++) {
        float cluster_angle = std::atan2(models[i].values[1], models[i].values[0]) + M_PI;
        // range of cluster_angle, inconveniently, is (0, 2*pi]. This turns it into [0, N_STAIR_ANGLE_BUCKETS).
        std::size_t bucket = static_cast<std::size_t>(std::ceil(cluster_angle * N_STAIR_ANGLE_BUCKETS / (2 * M_PI)) - 1);
        // leaky buckets -- each increments its neighbors too
//        const std::size_t prev_bucket = (bucket - 1) % N_STAIR_ANGLE_BUCKETS;
//        const std::size_t next_bucket = (bucket + 1) % N_STAIR_ANGLE_BUCKETS;
        stair_angle_buckets.at(bucket) += 1;
//        stair_angle_buckets.at(prev_bucket) += 1;
//        stair_angle_buckets.at(next_bucket) += 1;
    }

    for (std::size_t i = 0; i < clusters.size(); i++) {
        Eigen::Vector3f coef = modelToVector(models[i]);
    }

    std::vector<std::size_t> threshold_buckets;
    for (std::size_t i = 0; i < stair_angle_buckets.size(); i++) {
        if (stair_angle_buckets[i] >= clusters.size() / 5) {
            threshold_buckets.push_back(i);
        }
    }

    std::sort(threshold_buckets.begin(), threshold_buckets.end(),
              [&stair_angle_buckets](const std::size_t &a, const std::size_t &b) -> bool {
                  return stair_angle_buckets[a] < stair_angle_buckets[b];
              });

    ROS_DEBUG_STREAM("Got " << threshold_buckets.size() << " candidate stair angles");

    if (threshold_buckets.empty()) {
        ROS_DEBUG_STREAM("Abandoning stair detection attempt, stair angle could not be determined");
        return false;
    }

    // Make sure x points away from the robot
    tf::StampedTransform robot_pose_tf;
    try {
        tf_listener_.lookupTransform(filtered_cloud->header.frame_id, "leftFoot", ros::Time(0), robot_pose_tf);
    } catch (const tf::TransformException &e) {
        ROS_DEBUG_STREAM("Abandoning stair detection attempt because of a transform error when getting "
                                 << "robot position: \n" << e.what());
        return false;
    }
    Eigen::Vector3d robot_pos_d;
    tf::vectorTFToEigen(robot_pose_tf.getOrigin(), robot_pos_d);
    Eigen::Vector3f robot_pos = robot_pos_d.cast<float>();

    float best_score = std::numeric_limits<float>::infinity();
    Eigen::Affine3f best_stair_pose;
    for (const std::size_t &bucket : threshold_buckets) {
        // Refine the angle -- average of the vectors within 0.1 radian of the bucket angle (measured in xy plane only)
        double bucket_angle = bucket * 2 * M_PI / N_STAIR_ANGLE_BUCKETS - M_PI;
        Eigen::Vector3f bucket_dir = Eigen::AngleAxisf(bucket_angle, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitX();
//        Eigen::Vector3f refined_dir = Eigen::Vector3f::Zero();
//
//        int n_added_pos = 0, n_added_neg = 0;
//        for (std::size_t cluster_i = 0; cluster_i < clusters.size(); cluster_i++) {
//            const Eigen::Vector3f &model_vect = modelToVector(models[cluster_i]);
//            ROS_DEBUG_STREAM("Angle " << std::atan2(models[cluster_i].values[1], models[cluster_i].values[0])
//                                      << " and opposite "
//                                      << std::atan2(-models[cluster_i].values[1], -models[cluster_i].values[0]));
//            // test the angle and its opposite to ensure that 0 and 180 are considered adjacent
//            if (std::abs(std::atan2(models[cluster_i].values[1], models[cluster_i].values[0]) - bucket_angle) < 0.05) {
//                refined_dir += model_vect;
//                ROS_DEBUG_STREAM("Adding " << model_vect.transpose() << " to refined_dir (pos)");
//                n_added_pos++;
//            } else if (std::abs(std::atan2(-models[cluster_i].values[1], -models[cluster_i].values[0]) - bucket_angle) < 0.05) {
//                refined_dir += -1 * model_vect;
//                ROS_DEBUG_STREAM("Adding " << (-model_vect).transpose() << " to refined_dir (nex)");
//                n_added_neg++;
//            }
//        }
//
//        refined_dir.z() = 0; // Force direction to be in the xy plane
//        refined_dir.normalize();
        Eigen::Vector3f refined_dir = bucket_dir;

        // Try this angle and its opposite (the wrong one will return failure fairly quickly)
        Eigen::Affine3f stair_pose_pos, stair_pose_neg;
        float score_pos = estimateStairPose(filtered_cloud, clusters, refined_dir, robot_pos, stair_pose_pos);
//        ros::Duration(2).sleep();
        float score_neg = estimateStairPose(filtered_cloud, clusters, -refined_dir, robot_pos, stair_pose_neg);
//        ros::Duration(2).sleep();

        // Figure out which of these worked and update the current best if necessary
        float score = std::min(score_pos, score_neg);
        Eigen::Affine3f stair_pose = score_pos < score_neg ? stair_pose_pos : stair_pose_neg;
        if (score < best_score) {
            best_score = score;
            best_stair_pose = stair_pose;
        }
    }

    if (!std::isfinite(best_score)) {
        ROS_DEBUG_STREAM("Abandoning stair detection attempt, stair pose could not be determined");
        return false;
    }

    ROS_DEBUG_STREAM("Identified best stairs pose");

    // Now that we finally have a reliable stair pose (assumed to be the very center of the bottommost step), base the
    // pose of the individual steps off it
    std::vector<geometry_msgs::Pose> step_poses;

    // Add the first pose as the point on the ground right in front of the staircase. Note that because the robot is
    // standing on the walkway, not the ground, this pose doesn't follow the pattern used in the for loop
    Eigen::Translation3f base_pose_ground(-STEP_DEPTH, 0, 0); // changed this line to change offset \\\\\\\\\\~~~~~~~
    Eigen::Affine3f step_pose_ground = best_stair_pose * base_pose_ground;
    geometry_msgs::Pose step_pose_ground_ros;
    tf::poseEigenToMsg(step_pose_ground.cast<double>(), step_pose_ground_ros);
    step_poses.push_back(step_pose_ground_ros);

    // Add the remaining poses on the steps
    for (int i = 0; i < 9; i++) {
        Eigen::Translation3f base_pose((i + 0.5f) * STEP_DEPTH, 0, (i + 0.5f) * STEP_HEIGHT);
        Eigen::Affine3f step_pose = best_stair_pose * base_pose;

        geometry_msgs::Pose step_pose_ros;
        tf::poseEigenToMsg(step_pose.cast<double>(), step_pose_ros);
        step_poses.push_back(step_pose_ros);
    }

    visualization_msgs::MarkerArray ma;
    int marker_id_ctr = 0;
    float pose_m_r = RANDOM_STEP_ARROW_COLOR ? static_cast<float>(std::rand()) / RAND_MAX : 0;
    float pose_m_g = RANDOM_STEP_ARROW_COLOR ? static_cast<float>(std::rand()) / RAND_MAX : 1;
    float pose_m_b = RANDOM_STEP_ARROW_COLOR ? static_cast<float>(std::rand()) / RAND_MAX : 1;
    for (const geometry_msgs::Pose &pose : step_poses) {
        visualization_msgs::Marker stair_pose_m;
        pcl_conversions::fromPCL(filtered_cloud->header, stair_pose_m.header);
        stair_pose_m.ns = "stairs_pose" ;
        stair_pose_m.id = marker_id_ctr++;
        stair_pose_m.type = visualization_msgs::Marker::CUBE;
        stair_pose_m.pose = pose;
        stair_pose_m.scale.x = 0.05;
        stair_pose_m.scale.y = 0.05;
        stair_pose_m.scale.z = 0.05;
        stair_pose_m.color.r = pose_m_r;
        stair_pose_m.color.g = pose_m_g;
        stair_pose_m.color.b = pose_m_b;
        stair_pose_m.color.a = 1;
        ma.markers.push_back(stair_pose_m);
    }
    marker_pub_.publish(ma);

    detections_.push_back(step_poses);

    return true;
}

std::vector<pcl::PointIndices> stair_detector_2::findStairClusters(const Eigen::Vector3f &avg_normal,
                                                                   const PointCloud::ConstPtr &filtered_cloud) const {
    pcl::SACSegmentation<Point> sac;
    sac.setProbability(0.05);
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.04);
    sac.setAxis(avg_normal);
    sac.setEpsAngle(0.1);
    sac.setInputCloud(filtered_cloud);

    const auto num_points = filtered_cloud->size();
    auto remaining_indices = boost::make_shared<std::vector<int>>(num_points);
    iota(remaining_indices->begin(), remaining_indices->end(), 0);

    std::vector<pcl::PointIndices> clusters;
    int consecutive_failed_attempts = 0;

    auto kdt = boost::make_shared<pcl::search::KdTree<Point>>();
    while (remaining_indices->size() > 250 && consecutive_failed_attempts < 10) {
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;

        kdt->setInputCloud(filtered_cloud, remaining_indices);
        sac.setSamplesMaxDist(0.05, kdt);

        // Segment the largest planar component from the remaining cloud
        sac.setIndices(remaining_indices);
        sac.segment(inliers, coefficients);
        if (inliers.indices.size() == 0) {
            consecutive_failed_attempts += 1;
            continue;
        }

        std::vector<int> tmp_indices;
        std::set_difference(remaining_indices->begin(), remaining_indices->end(),
                            inliers.indices.begin(), inliers.indices.end(),
                            back_inserter(tmp_indices));

        remaining_indices->swap(tmp_indices);

        if (inliers.indices.size() > 250) {
            consecutive_failed_attempts = 0;
            ROS_DEBUG_STREAM("Categorized \t" << inliers.indices.size() << " pts, \t"
                                              << remaining_indices->size() << " remaining");
            clusters.push_back(std::move(inliers));
        } else {
            consecutive_failed_attempts =+ 1;
        }
    }

    return clusters;
}

void stair_detector_2::publishClusters(const PointCloud::ConstPtr &cloud, const std::vector<pcl::PointIndices> &clusters) const {
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

void stair_detector_2::publishNormals(const PointCloud &points, const NormalCloud &normals) const {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    pcl_conversions::fromPCL(normals.header, marker.header);
    marker.ns = "model_normals";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.001;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1;
    marker.color.a = 1;

    PointCloud::VectorType::const_iterator pt_it;
    NormalCloud::VectorType::const_iterator n_it;
    for (pt_it = points.begin(), n_it = normals.begin();
         pt_it != points.end() && n_it != normals.end();
         ++pt_it, ++n_it) {
        if (!pt_it->getVector3fMap().allFinite() || !n_it->getNormalVector3fMap().allFinite()) continue;

        const Eigen::Vector3f endpt = pt_it->getVector3fMap() + n_it->getNormalVector3fMap() * 0.05;

        geometry_msgs::Point ros_start, ros_end;
        ros_start.x = pt_it->x;
        ros_start.y = pt_it->y;
        ros_start.z =pt_it->z;
        ros_end.x = endpt.x();
        ros_end.y = endpt.y();
        ros_end.z = endpt.z();

        marker.points.push_back(ros_start);
        marker.points.push_back(ros_end);
    }
    markers.markers.push_back(marker);
    marker_pub_.publish(markers);
}


Eigen::Vector3f stair_detector_2::modelToVector(const pcl::ModelCoefficients &model) const {
    Eigen::Vector3f coef(model.values[0], model.values[1], model.values[2]);
    ROS_WARN_STREAM_COND(std::abs(1 - coef.norm()) > 0.01,
                         "Coefficients from RANSAC were not normalized (norm " << coef.norm() << ")");
    // normalize all directions to have positive y or, if y == 0, positive x
    if (coef.y() < 0 || (coef.y() == 0 && coef.x() < 0)) coef *= -1;
    return coef;
}

bool stair_detector_2::getDetections(std::vector<geometry_msgs::Pose> &detections) {
    if (detections_.size() > 3) {
        detections = detections_.back();
        ROS_INFO("Walking goal %f %f %f",detections[0].position.x,detections[0].position.y,detections[0].position.z);
        utils_.task3LogPub("stair_detector_2::getDetections x : " + std::to_string(detections[0].position.x)
                +  " y : " + std::to_string(detections[0].position.y) + " z : "  + std::to_string(detections[0].position.z));
        return true;
    }
    return false;
}

stair_detector_2::~stair_detector_2()
{
    pcl_sub_.shutdown();
}
