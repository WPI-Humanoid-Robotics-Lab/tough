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


// COPIED FROM PCL 1.8
template <typename Scalar> bool
threePlanesIntersection(const Eigen::Matrix<Scalar, 4, 1> &plane_a,
                        const Eigen::Matrix<Scalar, 4, 1> &plane_b,
                        const Eigen::Matrix<Scalar, 4, 1> &plane_c,
                        Eigen::Matrix<Scalar, 3, 1> &intersection_point,
                        double determinant_tolerance = 1e-6) {
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    // TODO: Using Eigen::HyperPlanes is better to solve this problem
    // Check if some planes are parallel
    Matrix3 normals_in_lines;

    for (int i = 0; i < 3; i++)
    {
        normals_in_lines (i, 0) = plane_a[i];
        normals_in_lines (i, 1) = plane_b[i];
        normals_in_lines (i, 2) = plane_c[i];
    }

    Scalar determinant = normals_in_lines.determinant ();
    if (fabs (determinant) < determinant_tolerance)
    {
        // det ~= 0
        PCL_DEBUG ("At least two planes are parralel.\n");
        return (false);
    }

    // Left part of the 3 equations
    Matrix3 left_member;

    for (int i = 0; i < 3; i++)
    {
        left_member (0, i) = plane_a[i];
        left_member (1, i) = plane_b[i];
        left_member (2, i) = plane_c[i];
    }

    // Right side of the 3 equations
    Vector3 right_member;
    right_member << -plane_a[3], -plane_b[3], -plane_c[3];

    // Solve the system
    intersection_point = left_member.fullPivLu ().solve (right_member);
    return (true);
}


solar_panel_detector_2::solar_panel_detector_2(ros::NodeHandle nh, const geometry_msgs::PoseStamped &rover_pose) :
        nh_(nh),
        rover_pose_(rover_pose),
        tf_listener_(nh),
        point_cloud_listener_(nh, "/leftFoot", "/left_camera_frame"),
        detection_success_(false)
{
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("solar_panel_detection_debug_pose", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("solar_panel_detection_markers", 1);
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

    // DEBUG ONLY only do 1 callback
//    pcl_sub_.shutdown();

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

//    points_pub_.publish(filtered_cloud);
//    publishNormals(*filtered_cloud, *filtered_normals);

    ROS_DEBUG_STREAM("Got normals");

    auto points_in_trailer = boost::make_shared<pcl::PointIndices>();
    if (!findPointsInsideTrailer(filtered_cloud, filtered_normals, *points_in_trailer)) {
        ROS_DEBUG("Abandoning solar panel detection attempt because the trailer could not be located");
        return;
    }

    ROS_DEBUG_STREAM("Got " << points_in_trailer->indices.size() << " points in trailer");

    geometry_msgs::PoseStamped solar_panel_pose;
    if (!findSolarPanelPose(filtered_cloud, filtered_normals, points_in_trailer, solar_panel_pose)) {
        ROS_DEBUG("Abandoning solar panel detection attempt because the panel could not be located");
        return;
    }

    ROS_INFO("Got solar panel pose");

    // solar_panel_pose is relative to rover_pose
    // round-trip through Eigen to transform it (thanks, ROS)
    Eigen::Affine3d rover_pose_eigen, solar_panel_pose_eigen;
    tf::poseMsgToEigen(rover_pose_.pose, rover_pose_eigen);
    tf::poseMsgToEigen(solar_panel_pose.pose, solar_panel_pose_eigen);

    Eigen::Affine3d absolute_pose_eigen = rover_pose_eigen * solar_panel_pose_eigen;
    geometry_msgs::PoseStamped absolute_pose;
    absolute_pose.header = rover_pose_.header;
    tf::poseEigenToMsg(absolute_pose_eigen, absolute_pose.pose);
    pose_pub_.publish(absolute_pose);

    detection_success_ = true;
    latest_detection_ = absolute_pose;

//    ros::shutdown(); // FOR DEBUGGING ONLY
}

bool solar_panel_detector_2::getPanelPose(geometry_msgs::PoseStamped &pose) const {
    if (detection_success_) {
        pose = latest_detection_;
    }

    return detection_success_;
}

solar_panel_detector_2::PointCloud::Ptr solar_panel_detector_2::prefilterCloud(const PointCloud::ConstPtr &cloud_raw) const {
    // Get trailer pose as an Eigen
    geometry_msgs::PoseStamped rover_pose_cloud_frame;
    try {
//        tf_listener_.waitForTransform(cloud_raw->header.frame_id, rover_pose_.header.frame_id, ros::Time(0), ros::Duration(5));
        tf_listener_.transformPose(cloud_raw->header.frame_id, rover_pose_, rover_pose_cloud_frame);
    } catch (const tf::TransformException &e) {
        ROS_WARN_STREAM("Skipping detection attempt: could not transform rover pose into the cloud frame \n" << e.what());
        return boost::make_shared<PointCloud>();
    }

//    pose_pub_.publish(rover_pose_cloud_frame);
    Eigen::Affine3d rover_pose;
    tf::poseMsgToEigen(rover_pose_cloud_frame.pose, rover_pose);

    // Crop the cloud to the trailer
    pcl::IndicesPtr points_in_trailer = boost::make_shared<vector<int>>();
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
//    points_pub_.publish(points_in_trailer_aligned);

    // Downsample the cloud -- using OctreePointCloudVoxelCentroid rather than VoxelGrid because VoxelGrid behaves badly
    // with large point clouds
    auto filtered_cloud = boost::make_shared<PointCloud>();
    filtered_cloud->header = cloud_raw->header;
    // anything less dense than 1cm causes voxelization artifacts to mess with normal estimation
    pcl::octree::OctreePointCloudVoxelCentroid<Point> oct(0.01);
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
    ne.setRadiusSearch(0.1);

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
    rgs.setMinClusterSize(200); // probably can be much bigger than 100
    rgs.setNumberOfNeighbours(30);
    rgs.setCurvatureThreshold(0.07);
//    rgs.setSmoothnessThreshold(45.0 * M_PI / 180.0);
    rgs.setSmoothModeFlag(false);
    rgs.setInputCloud(points);
    rgs.setInputNormals(normals);

    auto clusters = boost::make_shared<std::vector<pcl::PointIndices>>();
    rgs.extract(*clusters);

    // Sort clusters so the largest cluster that fits each wall/floor description is encountered first
    std::sort(clusters->begin(), clusters->end(), [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
        return a.indices.size() > b.indices.size();
    });

//    publishClusters(points, *clusters);

    // Find the 3 walls and the floor
    pcl::ModelCoefficients back_model, floor_model, right_model, left_model;
    bool back_found = false, floor_found = false, right_found = false, left_found = false;

    pcl::SACSegmentationFromNormals<Point, Normal> sac;
    sac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.04);
    sac.setInputCloud(points);
    sac.setInputNormals(normals);
    // indices set inside the loop

    Eigen::Vector4f overall_centroid;
    pcl::compute3DCentroid(*points, overall_centroid);

    for (const pcl::PointIndices &cluster : *clusters) {
        pcl::PointIndices::ConstPtr cluster_ptr(clusters, &cluster); // aliasing pointer
        sac.setIndices(cluster_ptr);

        pcl::ModelCoefficients model;
        pcl::PointIndices inliers;
        sac.segment(inliers, model);

        Eigen::Vector4f cluster_centroid;
        pcl::compute3DCentroid(*points, cluster, cluster_centroid);

        if (cluster_centroid.z() < overall_centroid.z() && std::abs(model.values[2]) > 0.85) {
            // low centroid and close-to-unit-z normal: floor
            if (!floor_found) {
                floor_found = true;
                floor_model = model;
            }
        } else if (cluster_centroid.x() > overall_centroid.x() && std::abs(model.values[0]) > 0.75 && std::abs(model.values[2]) < 0.1) {
            // far-forward centroid and close-to-unit-x normal: back wall
            if (!back_found) {
                back_found = true;
                back_model = model;
            }
        } else if (cluster_centroid.y() < overall_centroid.y() && std::abs(model.values[1]) > 0.75 && std::abs(model.values[2]) < 0.1) {
            // far-right centroid and close-to-unit-y normal: right wall
            if (!right_found) {
                right_found = true;
                right_model = model;
            }
        } else if (cluster_centroid.y() > overall_centroid.y() && std::abs(model.values[1]) > 0.75 && std::abs(model.values[2]) < 0.1) {
            // far-left centroid and close-to-unit-y normal: left wall
            if (!left_found) {
                left_found = true;
                left_model = model;
            }
        }

        if (floor_found && back_found && right_found && left_found) break;
    }

    if (!(floor_found && back_found && right_found && left_found)) {
        ROS_DEBUG_STREAM("Skipping detection attempt: could not detect floor and walls ("
                          << floor_found << ", "  << back_found << ", " << right_found << ", " << left_found << ")");
//        publishClusters(points, *clusters);
        return false;
    }


    Eigen::Map<Eigen::Vector4f> floor_plane(floor_model.values.data());
    Eigen::Map<Eigen::Vector4f> back_plane(back_model.values.data());
    Eigen::Map<Eigen::Vector4f> right_plane(right_model.values.data());
    Eigen::Map<Eigen::Vector4f> left_plane(left_model.values.data());

    Eigen::Vector3f back_left_corner, back_right_corner;
    if (!(
            threePlanesIntersection<float>(floor_plane, back_plane, right_plane, back_right_corner) &&
            threePlanesIntersection<float>(floor_plane, back_plane, left_plane, back_left_corner)
    )) {
        ROS_DEBUG_STREAM("Skipping detection attempt: detected floor and walls were not perpendicular");
        return false;
    }

    // Move y and z of the back right corner a little bit towards the centroid, and set its x to turn it into min_pt
    Eigen::Vector4f min_pt(
            0,
            back_right_corner.y() + 0.05f * (back_right_corner.y() < overall_centroid.y() ? 1 : -1),
            back_right_corner.z() + 0.05f * (back_right_corner.z() < overall_centroid.z() ? 1 : -1),
            1
    );

    // Move x and y of the back left corner a little bit towards the centroid, and set its z to turn it into max_pt
    Eigen::Vector4f max_pt(
        back_left_corner.x() + 0.10f * (back_left_corner.x() < overall_centroid.x() ? 1 : -1),
        back_left_corner.y() + 0.05f * (back_left_corner.y() < overall_centroid.y() ? 1 : -1),
        3,
        1
    );

    ROS_DEBUG_STREAM("Adjusted max_pt.x by " << 0.05f * (back_left_corner.x() < overall_centroid.x() ? 1 : -1));

    // Take the average of the 3 walls, rotated to align and normalized to point in the same direction, as the axis
    Eigen::Vector3f trailer_axis = (
        Eigen::Affine3f(Eigen::AngleAxisf(M_PI * (back_plane.x() > 0 ? 0 : 1), Eigen::Vector3f::UnitZ())) * back_plane.head<3>() +
        Eigen::Affine3f(Eigen::AngleAxisf(M_PI_2 * (right_plane.y() > 0 ? -1 : 1), Eigen::Vector3f::UnitZ())) * right_plane.head<3>() +
        Eigen::Affine3f(Eigen::AngleAxisf(M_PI_2 * (left_plane.y() > 0 ? -1 : 1), Eigen::Vector3f::UnitZ())) * left_plane.head<3>()
    );
    trailer_axis.normalize();

    if (std::abs(trailer_axis.z()) > 0.05) {
        ROS_DEBUG_STREAM("Skipping detection attempt: detected trailer axis was not in xy plane " << trailer_axis.transpose());
        return false;
    }

    Eigen::Affine3f trailer_axis_tf(
            Eigen::Quaternionf::FromTwoVectors(trailer_axis, Eigen::Vector3f::UnitX())
    );

//    geometry_msgs::PoseStamped trailer_axis_pose;
//    pcl_conversions::fromPCL(points->header, trailer_axis_pose.header);
//    tf::poseEigenToMsg(trailer_axis_tf.cast<double>(), trailer_axis_pose.pose);
//    pose_pub_.publish(trailer_axis_pose);

    // Finally, crop
    pcl::CropBox<Point> cropFilter;
    cropFilter.setInputCloud(points);
    cropFilter.setMin(trailer_axis_tf * min_pt);
    cropFilter.setMax(trailer_axis_tf * max_pt);
    cropFilter.setTransform(trailer_axis_tf);

    auto pts_inside_trailer_candidate = boost::make_shared<pcl::PointIndices>();
    cropFilter.filter(pts_inside_trailer_candidate->indices);

    // Smoke test: Make sure there is only 1 significant connected component. This prevents badly-conditioned data from
    // causing incorrect detection
    pcl::EuclideanClusterExtraction<Point> ece;
    ece.setMinClusterSize(10);
    ece.setClusterTolerance(0.02);
    ece.setInputCloud(points);
    ece.setIndices(pts_inside_trailer_candidate);

    std::vector<pcl::PointIndices> ece_clusters;
    ece.extract(ece_clusters);

    if (ece_clusters.size() != 1) {
        ROS_DEBUG_STREAM("Skipping detection attempt: expected there to be 1 object in the trailer, but found " << ece_clusters.size());
        return false;
    }

    pts_inside_trailer = *pts_inside_trailer_candidate;

    return true;
}

bool solar_panel_detector_2::findSolarPanelPose(const PointCloud::ConstPtr &points,
                                                const NormalCloud::Ptr &normals, // not ConstPtr because of PCL
                                                const pcl::PointIndices::ConstPtr &pts_inside_trailer,
                                                geometry_msgs::PoseStamped &solar_panel_pose) const {
    pcl::RegionGrowing<Point, Normal> rgs;
    rgs.setMinClusterSize(200); // probably can be much bigger than 100
    rgs.setNumberOfNeighbours(30);
    rgs.setCurvatureThreshold(0.05);
    rgs.setInputCloud(points);
    rgs.setInputNormals(normals);
    rgs.setIndices(pts_inside_trailer);

    auto clusters = boost::make_shared<std::vector<pcl::PointIndices>>();
    rgs.extract(*clusters);

    // Sort clusters so the largest cluster that fits the expected shape is encountered first
    std::sort(clusters->begin(), clusters->end(), [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
        return a.indices.size() > b.indices.size();
    });


    pcl::SACSegmentationFromNormals<Point, Normal> sac;
    sac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.04);
    sac.setInputCloud(points);
    sac.setInputNormals(normals);

    for (const auto &cluster : *clusters) {
        pcl::PointIndices::ConstPtr cluster_ptr(clusters, &cluster); // aliasing pointer
        sac.setIndices(cluster_ptr);

        pcl::PointIndices inliers;
        pcl::ModelCoefficients model;
        sac.segment(inliers, model);

        if (std::abs(model.values[2]) > 0.1) {
            ROS_DEBUG_STREAM("Abandoning cluster of size " << cluster.indices.size()
                             << " because its normal was not close to the xy plane");
            continue;
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*points, inliers, centroid);

        Eigen::Affine3f panel_side_tf = (
            Eigen::Translation3f(centroid.head<3>()) *
            Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), Eigen::Map<Eigen::Vector3f>(model.values.data()))
        );

        geometry_msgs::PoseStamped panel_side_pose;
        tf::poseEigenToMsg(panel_side_tf.cast<double>(), panel_side_pose.pose);
        pcl_conversions::fromPCL(points->header, panel_side_pose.header);
//        pose_pub_.publish(panel_side_pose);

        auto panel_cloud = boost::make_shared<PointCloud>();
        pcl::transformPointCloud(*points, pts_inside_trailer->indices, *panel_cloud, panel_side_tf.inverse());
        panel_cloud->header = points->header;
//        points_pub_.publish(panel_cloud);

        // Make sure the majority of the point cloud is in the expected area
        std::vector<int> panel_pts_in_box;
        Eigen::Vector4f expected_box_max(0.16f, 0.25f, 0.30f, 1);
        Eigen::Vector4f expected_box_min = -expected_box_max;
        expected_box_min[3] = 1;
        pcl::getPointsInBox(*panel_cloud, expected_box_min, expected_box_max, panel_pts_in_box);

        if (panel_pts_in_box.size() < panel_cloud->size() * 0.9) {
            ROS_DEBUG_STREAM("Abandoning cluster of size " << cluster.indices.size()
                                                           << " because it was too far outside the expected box");
            continue;
        }

        ROS_ERROR_STREAM_COND(panel_cloud->empty(), "Panel cloud was empty!");
        
        const float gap_center_z = findGapAlongAxis(panel_cloud, 2);
        if (std::isnan(gap_center_z)) {
            ROS_DEBUG_STREAM("Abandoning cluster of size " << cluster.indices.size()
                                                           << " because its handle gap couldn't be found (z axis)");
            continue;
        }

        const float gap_center_y = findGapAlongAxis(panel_cloud, 1);
        if (std::isnan(gap_center_y)) {
            ROS_DEBUG_STREAM("Abandoning cluster of size " << cluster.indices.size()
                                                           << " because its handle gap couldn't be found (y axis)");
            continue;
        }

        // Handle is connected to box in all cross sections along x. For findGapAlongAxis to work, it needs to be
        // limited to the center, in between the places where the handle connects to the box -- gap_center_y +/- ~2cm
        std::vector<int> panel_cloud_x_indices;
        Eigen::Vector4f handle_center_crop_min(-10, gap_center_y - 0.02f, -10, 1);
        Eigen::Vector4f handle_crop_max(10, gap_center_y + 0.02f, 10, 1);
        pcl::getPointsInBox(*panel_cloud, handle_center_crop_min, handle_crop_max, panel_cloud_x_indices);
        // Few enough points that duplicating the cloud is insignificant; also, some algorithms inside findGapAlongAxis
        // can't handle indices.
        auto panel_cloud_x = boost::make_shared<PointCloud>(*panel_cloud, panel_cloud_x_indices);

        const float gap_center_x = findGapAlongAxis(panel_cloud_x, 0);
        if (std::isnan(gap_center_x)) {
            ROS_DEBUG_STREAM("Abandoning cluster of size " << cluster.indices.size()
                                                           << " because its handle gap couldn't be found (x axis)");
            continue;
        }

        Eigen::Translation3f handle_center(gap_center_x, gap_center_y, gap_center_z);

        Eigen::Affine3f final_pose = panel_side_tf * handle_center;
        tf::poseEigenToMsg(final_pose.cast<double>(), solar_panel_pose.pose);
        pcl_conversions::fromPCL(points->header, solar_panel_pose.header);

//        pose_pub_.publish(solar_panel_pose);
//        points_pub_.publish(points);

        // If loop hasn't continued yet, pose detection was successful
        return true;
    }

    // If we didn't return yet, the panel wasn't found
    return false;
}

float solar_panel_detector_2::findGapAlongAxis(const PointCloud::ConstPtr &points, const int axis) const {
    // This function finds gaps (such as the gap between the handle and the main piece of the solar panel) along the
    // given axis, and returns the center of the longest gap. It finds the gaps by iterating over axial cross sections
    // of the input cloud and finding the connected components, if a cross-section has multiple connected components
    // then a gap exists in that cross section.

    Point min_pt, max_pt;
    pcl::getMinMax3D(*points, min_pt, max_pt);
    const float range_diff = 0.01;
    const float range_min = min_pt.data[axis];
    const float range_max = max_pt.data[axis];

    float longest_gap_start = NAN;
    float longest_gap_end = NAN;
    float current_gap_start = NAN;
    for (float cross_section_center = range_min; cross_section_center < range_max; cross_section_center += range_diff) {
        auto cross_section = boost::make_shared<pcl::PointIndices>();
        Eigen::Vector4f cross_section_min(-10, -10, -10, 1), cross_section_max(10, 10, 10, 1);
        cross_section_min[axis] = cross_section_center - range_diff/2;
        cross_section_max[axis] = cross_section_center + range_diff/2;
        pcl::getPointsInBox(*points, cross_section_min, cross_section_max, cross_section->indices);

        pcl::EuclideanClusterExtraction<Point> ece;
        ece.setMinClusterSize(10);
        ece.setClusterTolerance(0.02);
        ece.setInputCloud(points);
        ece.setIndices(cross_section);

        std::vector<pcl::PointIndices> clusters;
        ece.extract(clusters);

        if (clusters.size() >= 2 && std::isnan(current_gap_start)) {
            // Case 1: Beginning of a new gap
            current_gap_start = cross_section_center;
        } else if (clusters.size() < 2 && !std::isnan(current_gap_start)) {
            // Case 2: End of a gap
            if (cross_section_center - current_gap_start > 1.5 * range_diff && (std::isnan(longest_gap_start) ||
                    (cross_section_center - current_gap_start) > (longest_gap_end - longest_gap_start))) {
                // If this is the new longest gap, and it's more than 1 long (possible spurious result), record it
                longest_gap_start = current_gap_start;
                longest_gap_end = cross_section_center;
            }
            // Always reset the current gap
            current_gap_start = NAN;
        }

        // All other cases: just continue iteration
    }

    // This naturally returns NAN if there was no gap
    return (longest_gap_start + longest_gap_end) / 2;
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

void solar_panel_detector_2::publishNormals(const PointCloud &points, const NormalCloud &normals) const {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    pcl_conversions::fromPCL(normals.header, marker.header);
    marker.ns = "model_normals";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.002;
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

        const Eigen::Vector3f endpt = pt_it->getVector3fMap() + n_it->getNormalVector3fMap() * 0.02;

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
