//
// Created by will on 5/31/17.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <val_task3/stair_detector_2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>
#include <perception_common/perception_common_names.h>


#define N_STAIR_ANGLE_BUCKETS 180
#define STEP_DEPTH 0.24f
#define STEP_HEIGHT 0.2f

stair_detector_2::stair_detector_2(ros::NodeHandle nh) :
        nh_(nh),
        tf_listener_(nh),
        point_cloud_listener_(nh, "/leftFoot", "/left_camera_frame")
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_stair", 1);
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

    std::vector<pcl::PointIndices> clusters;
    Eigen::Vector3f stairs_dir;
    if (!estimateStairs(filtered_cloud, filtered_normals)) return;
}

std::size_t stair_detector_2::estimateStairPose(const PointCloud::ConstPtr &filtered_cloud,
                                                const std::vector<pcl::PointIndices> &step_clusters,
                                                const Eigen::Vector3f &stairs_dir, Eigen::Affine3f &stairs_pose) const {
    float stairs_angle = atan2(stairs_dir.y(), stairs_dir.x());

    vector<pcl::PointIndices> stair_clusters;
    auto step_shape = boost::make_shared<LabeledCloud>();
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
        // Make sure that step exists in the cluster list
        if (stair_clusters.size() <= step) stair_clusters.resize(step + 1);
        // Add this cluster's points to the given step (note that indices are guaranteed to be unique)
        copy(cluster.indices.begin(), cluster.indices.end(), back_inserter(stair_clusters.at(step).indices));

        step_shape->reserve(step_shape->size() + cluster.indices.size());
        Eigen::Affine3f step_tf(Eigen::Translation3f(-STEP_DEPTH * step, 0, -STEP_HEIGHT * step)
                                * Eigen::AngleAxisf(stairs_angle, Eigen::Vector3f::UnitZ()));
        for (const auto &idx : cluster.indices) {
            Point pt_tf = pcl::transformPoint(filtered_cloud->at(idx), step_tf);
            step_shape->push_back(LabeledPoint());
            step_shape->back().label = static_cast<uint32_t>(step);
            step_shape->back().x = pt_tf.x;
            step_shape->back().y = pt_tf.y;
            step_shape->back().z = pt_tf.z;
        }
    }

    const long n_steps_found = count_if(stair_clusters.begin(), stair_clusters.end(),
                                            [](const pcl::PointIndices &c) { return c.indices.size() > 0; });

    if (n_steps_found < 5) {
        ROS_DEBUG_STREAM("Candidate step pose abandoned because only " << n_steps_found << " steps were found");
        return 0;
    }

    // all steps should now be overlaid on top of each other, entirely between -0.1 < z < 0. (within some error)
    pcl::PassThrough<LabeledPoint> pt;
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-0.12f, 0.12f);
    pt.setInputCloud(step_shape);
    auto step_clipped_indices = boost::make_shared<pcl::PointIndices>();
    pt.filter(step_clipped_indices->indices);

    // find the largest eucliden segment -- it should be very dense and by far the largest
    pcl::EuclideanClusterExtraction<LabeledPoint> cluster;
    cluster.setClusterTolerance(0.02);
    cluster.setMinClusterSize(step_clipped_indices->indices.size() / 4);
    cluster.setInputCloud(step_shape);
    cluster.setIndices(step_clipped_indices);

    std::vector<pcl::PointIndices> shape_clusters;
    cluster.extract(shape_clusters);

    if (shape_clusters.empty()) {
        ROS_DEBUG_STREAM("Candidate step pose abandoned because the steps were not at the expected positions for this angle");
        return 0;
    }

    auto stair_cluster_iter = std::max_element(
            shape_clusters.begin(), shape_clusters.end(),
            [](const pcl::PointIndices &a, const pcl::PointIndices &b) { return a.indices.size() < b.indices.size(); }
    );
    auto stair_cluster = boost::make_shared<pcl::PointIndices>();
    stair_cluster->indices = std::move(stair_cluster_iter->indices);

    ROS_DEBUG_STREAM("Biggest cluster had " << stair_cluster->indices.size() << " (" << shape_clusters.size() << " clusters)");

    // PCA to get centroid and improved estimate of the angle
    pcl::PCA<LabeledPoint> pca;
    pca.setInputCloud(step_shape);
    pca.setIndices(stair_cluster);

    stairs_pose.translation() = pca.getMean().head<3>();

    // Rotation of PCA
    Eigen::Matrix3f rot_mat = pca.getEigenVectors();
    
    // Reorder principal components into a valid rotation matrix
    stairs_pose.linear().col(0) = (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
    stairs_pose.linear().col(2) = rot_mat.col(1);
    stairs_pose.linear().col(1) = rot_mat.col(0);

//    // Make sure x points away from the robot
//    geometry_msgs::PointStamped foot_foot_frame, foot_world;
//    foot_foot_frame.header.stamp = ros::Time(0);
//    foot_foot_frame.header.frame_id = "leftFoot";
//    try {
//        tf_listener_.transformPoint(filtered_cloud->header.frame_id, foot_foot_frame, foot_world);
//    } catch (const tf::TransformException &e) {
//        ROS_DEBUG("Candidate step pose abandoned because the robot foot position could not be cound");
//        return 0;
//    }
//
//    if ((stairs_pose.inverse() * Eigen::Vector3f(foot_world.point.x, foot_world.point.y, foot_world.point.z)).x() > 0) {
//        ROS_DEBUG("Flipping X to point away from robot");
//        stairs_pose.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
//    }

    // Make sure z points up and not down
    if ((stairs_pose.linear() * Eigen::Vector3f::UnitZ()).z() < 0) {
        ROS_DEBUG("Flipping Z to point up");
        stairs_pose.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    }

//    this->points_pub_.publish(step_shape);

    return stair_cluster->indices.size();
}

stair_detector_2::PointCloud::Ptr stair_detector_2::prefilterCloud(const PointCloud::ConstPtr &cloud_raw) const {
    // Crop the cloud to the height of the staircase
    pcl::IndicesPtr points_in_room = boost::make_shared<vector<int>>();
    pcl::CropBox<Point> cropFilter;
    cropFilter.setInputCloud(cloud_raw);
    cropFilter.setMin({-8.f, -8.f, 0.02f, 1});
    cropFilter.setMax({8.f, 8.f, 2.f, 1});

    cropFilter.filter(*points_in_room);

    // Downsample the cloud1
    auto filtered_cloud = boost::make_shared<PointCloud>();
    filtered_cloud->header = cloud_raw->header;
    pcl::octree::OctreePointCloudVoxelCentroid<Point> oct(0.02);
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
    ne.setRadiusSearch(0.03);

    auto filtered_normals = boost::make_shared<NormalCloud>();
    filtered_normals->header = filtered_cloud->header;
    ne.compute(*filtered_normals);

    return filtered_normals;
}

bool stair_detector_2::estimateStairs(const PointCloud::ConstPtr &filtered_cloud,
                                      const NormalCloud::ConstPtr &filtered_normals) const {
    // Use RANSAC to find _vertical_ planes -- the horizontal ones are what we want, but are much less likely to be
    // seen by the multisense
    pcl::SACSegmentationFromNormals<Point, Normal> nsac;
    nsac.setOptimizeCoefficients(true);
    nsac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    nsac.setMethodType(pcl::SAC_RANSAC);
    nsac.setDistanceThreshold(0.06);
//    nsac.setProbability(0.80);
//    nsac.setNormalDistanceWeight(0.5);
    nsac.setInputCloud(filtered_cloud);
    nsac.setInputNormals(filtered_normals);

    const auto num_points = filtered_cloud->size();
    auto remaining_indices = boost::make_shared<std::vector<int>>(num_points);
    iota(remaining_indices->begin(), remaining_indices->end(), 0);

    std::vector<pcl::PointIndices> clusters;
    std::vector<pcl::ModelCoefficients> models;
    int consecutive_failed_attempts = 0;

    auto kdt = boost::make_shared<pcl::search::KdTree<Point>>();
    while (remaining_indices->size() > 400 && consecutive_failed_attempts < 5) {
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;

        if (consecutive_failed_attempts == 0) {
            kdt->setInputCloud(filtered_cloud, remaining_indices);
            nsac.setSamplesMaxDist(0.05, kdt);
        }

        // Segment the largest planar component from the remaining cloud
        nsac.setIndices(remaining_indices);
        nsac.segment(inliers, coefficients);
        if (inliers.indices.size() == 0) {
            consecutive_failed_attempts += 1;
            ROS_DEBUG_STREAM("Failed " << consecutive_failed_attempts << (consecutive_failed_attempts == 1 ? " time" : " times"));
        }

        vector<int> tmp_indices;
        set_difference(remaining_indices->begin(), remaining_indices->end(),
                       inliers.indices.begin(), inliers.indices.end(),
                       back_inserter(tmp_indices));

        remaining_indices->swap(tmp_indices);

        Eigen::Vector3f coef = modelToVector(coefficients);

        if (std::abs(coef.z()) < 0.1 && inliers.indices.size() > 250) {
            consecutive_failed_attempts = 0;
            ROS_DEBUG_STREAM("Categorized \t" << inliers.indices.size() << " pts, \t"
                                              << remaining_indices->size() << " remaining");
            models.push_back(std::move(coefficients));
            clusters.push_back(std::move(inliers));
        } else {
            consecutive_failed_attempts += 1;
            ROS_DEBUG_STREAM("Failed " << consecutive_failed_attempts << (consecutive_failed_attempts == 1 ? " time" : " times"));
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
        const std::size_t prev_bucket = (bucket - 1) % N_STAIR_ANGLE_BUCKETS;
        const std::size_t next_bucket = (bucket + 1) % N_STAIR_ANGLE_BUCKETS;
        stair_angle_buckets.at(bucket) += 1; // TODO: Angles farther from the xy plane contribute less
        stair_angle_buckets.at(prev_bucket) += 1;
        stair_angle_buckets.at(next_bucket) += 1;
    }

    publishClusters(filtered_cloud, clusters);

    visualization_msgs::MarkerArray ma;

    for (std::size_t i = 0; i < clusters.size(); i++) {
        Eigen::Vector3f coef = modelToVector(models[i]);

        visualization_msgs::Marker normals_marker;
        pcl_conversions::fromPCL(filtered_cloud->header, normals_marker.header);
        normals_marker.ns = "normals_arrows";
        normals_marker.id = static_cast<int>(i);
        normals_marker.type = visualization_msgs::Marker::ARROW;
        normals_marker.points.push_back(geometry_msgs::Point());
        normals_marker.points.back().x = 0;
        normals_marker.points.back().y = 0;
        normals_marker.points.back().z = 0;
        normals_marker.points.push_back(geometry_msgs::Point());
        normals_marker.points.back().x = coef.x();
        normals_marker.points.back().y = coef.y();
        normals_marker.points.back().z = coef.z();
        normals_marker.scale.x = 0.01;
        normals_marker.scale.y = 0.02;
        normals_marker.color.r = 0;
        normals_marker.color.g = 0;
        normals_marker.color.b = 0.5;
        normals_marker.color.a = 1;
        ma.markers.push_back(normals_marker);
    }

    for (std::size_t i = clusters.size(); i < 500; i++) {
        visualization_msgs::Marker delete_normals_marker;
        pcl_conversions::fromPCL(filtered_cloud->header, delete_normals_marker.header);
        delete_normals_marker.ns = "normals_arrows";
        delete_normals_marker.id = static_cast<int>(i);
        delete_normals_marker.action = visualization_msgs::Marker::DELETE;
        ma.markers.push_back(delete_normals_marker);
    }

    std::vector<std::size_t> threshold_buckets;
    for (std::size_t i = 0; i < stair_angle_buckets.size(); i++) {
        if (stair_angle_buckets[i] >= clusters.size() / 3) {
            threshold_buckets.push_back(i);
        }
    }

    std::sort(threshold_buckets.begin(), threshold_buckets.end(),
              [&stair_angle_buckets](const std::size_t &a, const std::size_t &b) -> bool {
                  return stair_angle_buckets[a] < stair_angle_buckets[b];
              });

    if (threshold_buckets.empty()) {
        ROS_DEBUG_STREAM("Abandoning stair detection attempt, stair angle could not be determined");
        marker_pub_.publish(ma);
        return false;
    }

    std::size_t max_n_supporting_samples = 0;
    Eigen::Affine3f best_stair_pose;
    int normals_marker_counter = 0;
    for (const std::size_t &bucket : threshold_buckets) {
        // Refine the angle -- average of the vectors within 0.1 radian of the bucket angle (measured in xy plane only)
        double bucket_angle = bucket * 2 * M_PI / N_STAIR_ANGLE_BUCKETS - M_PI;
        Eigen::Vector3f bucket_dir = Eigen::AngleAxisf(bucket_angle, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f::UnitX();
        Eigen::Vector3f refined_dir = Eigen::Vector3f::Zero();

        int n_added_pos = 0, n_added_neg = 0;
        for (std::size_t cluster_i = 0; cluster_i < clusters.size(); cluster_i++) {
            // test the angle and its opposite to ensure that 0 and 180 are considered adjacent
            if (std::abs(std::atan2(models[cluster_i].values[1], models[cluster_i].values[0]) - bucket_angle) < 0.05) {
                refined_dir += modelToVector(models[cluster_i]);
                n_added_pos++;
            } else if (std::abs(std::atan2(-models[cluster_i].values[1], -models[cluster_i].values[0]) - bucket_angle) < 0.1) {
                refined_dir += -1 * modelToVector(models[cluster_i]);
                n_added_neg++;
            }
        }

        refined_dir.z() = 0; // Force direction to be in the xy plane
        refined_dir.normalize();

        // Try this angle and its opposite (the wrong one will return failure fairly quickly)
        Eigen::Affine3f stair_pose_pos, stair_pose_neg;
        std::size_t n_supporting_samples_pos = estimateStairPose(filtered_cloud, clusters, refined_dir, stair_pose_pos);
        std::size_t n_supporting_samples_neg = estimateStairPose(filtered_cloud, clusters, -refined_dir, stair_pose_neg);

        // Figure out which of these worked and update the current best if necessary
        std::size_t n_supporting_samples = std::max(n_supporting_samples_pos, n_supporting_samples_neg);
        Eigen::Affine3f stair_pose = n_supporting_samples_pos < n_supporting_samples_neg ? stair_pose_neg : stair_pose_pos;
        if (n_supporting_samples > max_n_supporting_samples) {
            max_n_supporting_samples = n_supporting_samples;
            best_stair_pose = stair_pose;
        }

        Eigen::Vector3f stair_pose_dir = stair_pose.linear() * Eigen::Vector3f::UnitX();

        visualization_msgs::Marker normals_marker;
        pcl_conversions::fromPCL(filtered_cloud->header, normals_marker.header);
        normals_marker.ns = "normals_arrows";
        normals_marker.id = static_cast<int>(normals_marker_counter++);
        normals_marker.type = visualization_msgs::Marker::ARROW;
        normals_marker.points.push_back(geometry_msgs::Point());
        normals_marker.points.back().x = 0;
        normals_marker.points.back().y = 0;
        normals_marker.points.back().z = 0;
        normals_marker.points.push_back(geometry_msgs::Point());
        normals_marker.points.back().x = stair_pose_dir.x();
        normals_marker.points.back().y = stair_pose_dir.y();
        normals_marker.points.back().z = stair_pose_dir.z();
        normals_marker.scale.x = 0.01;
        normals_marker.scale.y = 0.02;
        normals_marker.color.r = 0;
        normals_marker.color.g = 0.5;
        normals_marker.color.b = 0;
        normals_marker.color.a = 1;
        ma.markers.push_back(normals_marker);
    }

    if (max_n_supporting_samples == 0) {
        ROS_DEBUG_STREAM("Abandoning stair detection attempt, stair pose could not be determined");
        marker_pub_.publish(ma);
        return false;
    }

    ROS_DEBUG_STREAM("Identified best stairs pose");

    // Now that we finally have a reliable stair pose (assumed to be the very center of the bottommost step), base the
    // pose of the individual steps off it
    std::vector<geometry_msgs::Pose> step_poses;
    for (int i = 0; i < 9; i++) {
        Eigen::Translation3f base_pose((i + 0.5f) * STEP_DEPTH, 0, (i + 0.5f) * STEP_HEIGHT);
        Eigen::Affine3f step_pose = best_stair_pose * base_pose;

        step_poses.push_back(geometry_msgs::Pose());
        step_poses.back().position.x = step_pose.translation().x();
        step_poses.back().position.y = step_pose.translation().y();
        step_poses.back().position.z = step_pose.translation().z();

        Eigen::Quaternionf step_quat(step_pose.linear());
        step_poses.back().orientation.x = step_quat.x();
        step_poses.back().orientation.y = step_quat.y();
        step_poses.back().orientation.z = step_quat.z();
        step_poses.back().orientation.w = step_quat.w();

        visualization_msgs::Marker stair_pose_m;
        pcl_conversions::fromPCL(filtered_cloud->header, stair_pose_m.header);
        stair_pose_m.ns = "stairs_pose" ;
        stair_pose_m.id = i;
        stair_pose_m.type = visualization_msgs::Marker::ARROW;
        stair_pose_m.pose = step_poses.back();
        stair_pose_m.scale.x = 1;
        stair_pose_m.scale.y = 0.05;
        stair_pose_m.scale.z = 0.05;
        stair_pose_m.color.r = 0;
        stair_pose_m.color.g = 1;
        stair_pose_m.color.b = 1;
        stair_pose_m.color.a = 1;
        ma.markers.push_back(stair_pose_m);
    }

    marker_pub_.publish(ma);

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

        vector<int> tmp_indices;
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

Eigen::Vector3f stair_detector_2::modelToVector(const pcl::ModelCoefficients &model) const {
    Eigen::Vector3f coef(model.values[0], model.values[1], model.values[2]);
    ROS_WARN_STREAM_COND(std::abs(1 - coef.norm()) > 0.01,
                         "Coefficients from RANSAC were not normalized (norm " << coef.norm() << ")");
    // normalize all directions to have positive y or, if y == 0, positive x
    if (coef.y() < 0 || (coef.y() == 0 && coef.x() < 0)) coef *= -1;
    return coef;
}

bool stair_detector_2::findStair() {
    return false;
}

stair_detector_2::~stair_detector_2()
{
    pcl_sub_.shutdown();
}
