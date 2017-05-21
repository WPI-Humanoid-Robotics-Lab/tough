// standard includes
#include <thread>
#include <algorithm>

// Library includes
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <Eigen/StdVector>

// Local includes
#include "val_task3/table_detector.h"
#include <visualization_msgs/Marker.h>
#include "val_common/val_common_names.h"

#define DISABLE_DRAWINGS false
//#define DISABLE_TRACKBAR true

table_detector::table_detector(ros::NodeHandle nh) : nh_(nh), point_cloud_listener_(nh, "/leftFoot", "/left_camera_frame")
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_table",1);
    points_pub_ = nh_.advertise<table_detector::PointCloud>("table_detection_debug_points",1);
    pcl_sub_ =  nh.subscribe("/field/assembled_cloud2", 10, &table_detector::cloudCB, this);
    ROS_DEBUG("Table detector setup finished");
}

void table_detector::cloudCB(const table_detector::PointCloud::ConstPtr &cloud) {
    ROS_DEBUG_STREAM("Got point cloud of size " << cloud->size());
    // Run it through a z filter to approximately the height of the table
    pcl::PassThrough<table_detector::Point> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("z");

    auto points_at_table_height = boost::make_shared<pcl::PointIndices>();
    auto points_at_leg_height = boost::make_shared<pcl::PointIndices>();
    pass.setFilterLimits(0.7, 0.9); // TODO: Extract constants
    pass.filter(points_at_table_height->indices);
    pass.setFilterLimits(0.4, 0.6); // TODO: Extract constants
    pass.filter(points_at_leg_height->indices);

    // Perform euclidean clustering to find candidates
    auto candidates = boost::make_shared<std::vector<pcl::PointIndices>>();
    pcl::EuclideanClusterExtraction<table_detector::Point> ec;
    ec.setClusterTolerance(0.04); // meters
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setInputCloud(cloud);
    ec.setIndices(points_at_table_height);
    ec.extract(*candidates);
    // Prep for use as leg clusterer
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(200);
    ec.setIndices(boost::make_shared<pcl::PointIndices>()); // clear indices

    ROS_DEBUG_STREAM("Got " << candidates->size() << " candidate planes");

    // Use RANSAC on each candidate with PERPENDICULAR_PLANE w/r/t z-axis to get surface height
    pcl::SACSegmentation<table_detector::Point> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis({0.0, 0.0, 1.0});
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);

    pcl::SACSegmentation<table_detector::Point> leg_seg;
    leg_seg.setOptimizeCoefficients(true);
    leg_seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    leg_seg.setAxis({0.0, 0.0, 1.0});
    leg_seg.setMethodType(pcl::SAC_RANSAC);
    leg_seg.setDistanceThreshold(0.02);

    pcl::ProjectInliers<table_detector::Point> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);

    pcl::ConvexHull<table_detector::Point> chull;
    chull.setDimension(2);
    chull.setComputeAreaVolume(true);

    // Check each candidate for table legs:
    // Find points 20 +/- 2.5cm below the table surface and within the concave hull
    // Group points into "leg" and "not leg" based on proximity to rectangle corner
    // If there are enough "leg" points and not many "not leg" points, then approve the candidate

    // Check each remaining candidate for items on the table:
    // Chop the area 2cm-10cm above the table and within the table boundary
    // Euclidean segment it
    // If there are two segments, it is good

    visualization_msgs::MarkerArray markers;

    int marker_id = 0;
    int eliminated_candidates = 0;
    for (const auto &candidate_indices : *candidates) {
        // seg takes shared_ptr to the indices, so make one that refers back to the shared_ptr to the vector of indices
        const pcl::PointIndices::ConstPtr cluster_ptr(candidates, &candidate_indices);
        seg.setIndices(cluster_ptr);

        auto inliers = boost::make_shared<pcl::PointIndices>();
        auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
        seg.segment(*inliers, *coefficients);

        proj.setModelCoefficients(coefficients);
        proj.setIndices(inliers);
        auto cloud_projected = boost::make_shared<table_detector::PointCloud>();
        proj.filter(*cloud_projected);

        proj.setIndices(points_at_leg_height);
        auto legs_projected = boost::make_shared<table_detector::PointCloud>();
        proj.filter(*legs_projected);

        // Create a Convex Hull representation of the projected inliers
        auto cloud_hull = boost::make_shared<table_detector::PointCloud>();
        chull.setInputCloud(cloud_projected);
        chull.reconstruct(*cloud_hull);

        const double area = chull.getTotalArea();
        if (std::abs(area - 1.71) > 0.5) {
            ROS_DEBUG_STREAM("Candidate " << (++eliminated_candidates) << " eliminated based on area (" << area << ")");
            continue;
        }

        // chull gives implicitly closed shapes, this explicitly closes it
        cloud_hull->push_back(cloud_hull->front());

        // Calculate the lines
        using HullLine = std::tuple<Eigen::Vector3f /* point */, Eigen::Vector3f /* direction */, float>;
        std::vector<HullLine> hull_lines;
        std::transform(cloud_hull->begin(), cloud_hull->end() - 1, cloud_hull->begin() + 1, std::back_inserter(hull_lines),
            [](const table_detector::Point &pt1, const table_detector::Point &pt2) {
                const Eigen::Vector3f &v1 = pt1.getVector3fMap();
                const Eigen::Vector3f &v2 = pt2.getVector3fMap();
                const Eigen::Vector3f &dir = v2 - v1;
                return std::make_tuple(v1, dir, dir.squaredNorm());
            }
        );

        // Leg detection: For every point at leg height that's close enough to the hull, project it onto the hull. Then
        // cluster those projections, filter for "looks like a right angle", and assume those are the legs. Discard the
        // candidate if there is only 1 cluster, because then it's probably a box (or a table we haven't seen enough
        // of yet).
        auto leg_candidate_pts = boost::make_shared<table_detector::PointCloud>();
        Eigen::Vector4f centroid, max_pt;
        pcl::compute3DCentroid(*cloud_hull, centroid);
        pcl::getMaxDistance(*cloud_hull, centroid, max_pt);
        const auto dist_threshold_sqr = std::pow(0.1 + (max_pt - centroid).norm(), 2);
        for (const auto &pt : legs_projected->points) {
            // fast filter by distance to centroid
            if ((pt.getVector4fMap() - centroid).squaredNorm() > dist_threshold_sqr) continue;

            std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> pt_projected;
            std::transform(hull_lines.begin(), hull_lines.end(), std::back_inserter(pt_projected),
                [&pt](const HullLine &line) {
                    const Eigen::Vector3f &line_origin = std::get<0>(line);
                    const Eigen::Vector3f &line_dir = std::get<1>(line).normalized();
                    const float &line_length = std::sqrt(std::get<2>(line));
                    const float &scalar_projection = (pt.getVector3fMap() - line_origin).dot(line_dir);
                    const auto constrained_scalar_proj = std::max(0.0f, std::min(line_length, scalar_projection));

                    return line_origin + constrained_scalar_proj * line_dir;
                }
            );

            const auto closest_projection = std::min_element(pt_projected.begin(), pt_projected.end(),
                 [&pt](const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
                     return (pt.getVector3fMap() - a).squaredNorm() < (pt.getVector3fMap() - b).squaredNorm();
                 });

            if ((pt.getVector3fMap() - *closest_projection).squaredNorm() > 0.01) continue;

            leg_candidate_pts->points.emplace_back();
            leg_candidate_pts->points.back().x = closest_projection->x();
            leg_candidate_pts->points.back().y = closest_projection->y();
            leg_candidate_pts->points.back().z = closest_projection->z();
        }

        pcl::search::KdTree<table_detector::Point> leg_candidate_search;
        leg_candidate_search.setInputCloud(leg_candidate_pts);

        // Simultaneously identify and discretize continuous between-corner segments
        std::vector<int> ki_unused;
        std::vector<float> ksd_unused;
        int consecutive_leg_pts = 0;
        auto non_leg_points = boost::make_shared<table_detector::PointCloud>();
        auto non_leg_clusters = boost::make_shared<std::vector<pcl::PointIndices>>();
        non_leg_clusters->emplace_back(); // Start a cluster for the first loop to fill
        for (const auto &line : hull_lines) {
            const Eigen::Vector3f &line_origin = std::get<0>(line);
            const Eigen::Vector3f &line_dir = std::get<1>(line).normalized();
            const float &line_length = std::sqrt(std::get<2>(line));

            for (float current_length = 0; current_length < line_length; current_length += 0.01) {
                const auto &pt_eigen = line_origin + line_dir * current_length;
                table_detector::Point pt_pcl;
                pt_pcl.x = pt_eigen.x();
                pt_pcl.y = pt_eigen.y();
                pt_pcl.z = pt_eigen.z();

                if (leg_candidate_search.radiusSearch(pt_pcl, 0.02, ki_unused, ksd_unused, 1) > 0) {
                    // Then there was a leg point nearby
                    consecutive_leg_pts++;
                } else {
                    // No leg point nearby -> this belongs in non_leg_points
                    if (consecutive_leg_pts > 4) {
                        // Then this is the first non-leg point in a while and should start a new cluster
                        if (non_leg_clusters->back().indices.size() < 5) {
                            // Then the previous cluster was too small, so just clear it and re-use it as the new one
                            ROS_DEBUG("New cluster in place of the last one");
                            non_leg_clusters->back().indices.clear();
                        } else {
                            // Then the previous cluster was good, so construct a new empty one at the back
                            ROS_DEBUG("New cluster");
                            non_leg_clusters->emplace_back();
                        }
                    }
                    non_leg_clusters->back().indices.push_back(non_leg_points->size());
                    non_leg_points->push_back(pt_pcl);
                    consecutive_leg_pts = 0;
                }
            }
        }

        if (non_leg_clusters->front().indices.empty()) {
            ROS_DEBUG_STREAM("Candidate " << (++eliminated_candidates)
                                          << " eliminated because it has \"legs\" around the entire perimeter");
            continue;
        }

        // No guarantee the first point at the iteration was at the beginning of one of the sides, so merge the first
        // and last clusters if their first and last points, respectively, are very close
        auto &front_indices = non_leg_clusters->front().indices;
        auto &back_indices = non_leg_clusters->back().indices;
        if (!non_leg_clusters->back().indices.empty() && // would otherwise segfault if last cluster is empty
            (non_leg_points->at(front_indices.front()).getVector3fMap() -
             non_leg_points->at(back_indices.back()).getVector3fMap()).squaredNorm() <= 0.01*0.01) {
            // Move contents of front to the end of back, then move back onto front, then delete the invalid back
            // I do it this way to maintain point ordering with minimal work, in case that turns out to be useful later
            std::move(front_indices.begin(), front_indices.end(), std::back_inserter(back_indices));
            // indices is the only meaningful part of PointIndices in this case, and it benefits from std::move while
            // PointIndices has not yet implemented a move constructor
            non_leg_clusters->front().indices = std::move(non_leg_clusters->back().indices);
            non_leg_clusters->pop_back(); // Deletes the moved-from object
        }

        // The last cluster might not have gone through the size check, so do it here
        if (non_leg_clusters->back().indices.size() < 5) {
            non_leg_clusters->pop_back();
        }

        if (non_leg_clusters->size() <= 3) {
            ROS_DEBUG_STREAM("Candidate " << (++eliminated_candidates)
                                          << " eliminated because it has fewer than 2 detected legs ("
                                          << non_leg_clusters->size() << ")");
            continue;
        }
        ROS_DEBUG_STREAM("Candidate had " << non_leg_clusters->size() << " detected sides");

        // Try RANSAC with a line model on each cluster, and use the best match to determine the table's yaw
        leg_seg.setInputCloud(non_leg_points);
        double best_variance = std::numeric_limits<double>::infinity();
        pcl::ModelCoefficients best_model;
        for (const auto &edge_cluster : *non_leg_clusters) {
            // Make an aliasing pointer for the cluster that aliases the pointer to the vector of clusters
            const auto edge_cluster_ptr = pcl::PointIndices::ConstPtr(non_leg_clusters, &edge_cluster);
            leg_seg.setIndices(edge_cluster_ptr);

            pcl::PointIndices edge_inliers;
            pcl::ModelCoefficients edge_model;
            leg_seg.segment(edge_inliers, edge_model);

            if (edge_inliers.indices.size() < edge_cluster.indices.size() * 0.95) { // edges should be very straight
                continue;
            }

            const double variance = leg_seg.getModel()->computeVariance();
            if (variance < best_variance) {
                best_variance = variance;
                best_model = edge_model;
            }
        }

        // The table z axis is stored in coefficients and the x axis in best_model (those variables were not
        // named very well). Together they fully describe a 3d rotation to align the coordinate system to the table.
        // (ModelCoefficients' values 0..2 is the plane's unit normal and value 3 is distance, not needed here)
        Eigen::Vector3f table_axis_x(best_model.values[0], best_model.values[1], best_model.values[2]);
        Eigen::Vector3f table_axis_z(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Affine3f table_axis_align_tf;
        table_axis_align_tf.linear() << table_axis_x, table_axis_z.cross(table_axis_x), table_axis_z;
        // Leave translation at its default of 0 -- I don't care about it

        ROS_DEBUG_STREAM("\n" << table_axis_align_tf.linear());
        // NOTE: CURRENT TASK: results are all wrong because SACMODEL_PARALLEL_PLANE actually gives you a perpendicular
        // plane. Solution: Change to SACMODEL_LINE and set table_axis_x to the rejection (opposite of projection) of
        // the line described by best_model onto table_axis_y (it will need to be normalized and probably will need a
        // check to make sure isn't pointing in an wildly different direction). This will also fix the pretty big
        // problem that currently table_axis_x and table_axis_z are not perfectly orthogonal since both have tolerances.

        // Transform the table inliers, from a very long time ago, to the new coordinate system using the inverse
        table_detector::PointCloud table_pts_aligned;
        pcl::transformPointCloud(*cloud, *inliers, table_pts_aligned, table_axis_align_tf.inverse());

        // Finally we can get the bounding box via getMinMax3d
        table_detector::Point bb_min, bb_max;
        pcl::getMinMax3D(table_pts_aligned, bb_min, bb_max);
        const auto bb_center = (bb_min.getVector3fMap() + bb_max.getVector3fMap())/2;
        const auto bb_center_worldframe = table_axis_align_tf * bb_center;
        const Eigen::Quaternionf bb_orientation(table_axis_align_tf.linear());

        leg_candidate_pts->header = cloud->header;
        points_pub_.publish(leg_candidate_pts);

#if !DISABLE_DRAWINGS
        // HULL
        auto hull_marker = visualization_msgs::Marker();
        pcl_conversions::fromPCL(cloud->header, hull_marker.header);
        hull_marker.ns = "table_candidates";
        hull_marker.id = marker_id++;
        hull_marker.type = visualization_msgs::Marker::LINE_STRIP;
        hull_marker.scale.x = 0.01; // segment width in m
        hull_marker.color.r = 1;
        hull_marker.color.g = 0;
        hull_marker.color.b = 0;
        hull_marker.color.a = 1;

        for (const auto &pt : cloud_hull->points) {
            hull_marker.points.emplace_back();
            hull_marker.points.back().x = pt.x;
            hull_marker.points.back().y = pt.y;
            hull_marker.points.back().z = pt.z;
        }

        // LINE_STRIP doesn't automatically close
        hull_marker.points.emplace_back();
        hull_marker.points.back().x = cloud_hull->points.front().x;
        hull_marker.points.back().y = cloud_hull->points.front().y;
        hull_marker.points.back().z = cloud_hull->points.front().z;

        markers.markers.push_back(hull_marker);

        // CUBE
        auto cube_marker = visualization_msgs::Marker();
        pcl_conversions::fromPCL(cloud->header, cube_marker.header);
        cube_marker.ns = "table_candidates";
        cube_marker.id = marker_id++;
        cube_marker.type = visualization_msgs::Marker::CUBE; // It's called cube, but it can be a rectangular prism
        cube_marker.pose.position.x = bb_center_worldframe.x();
        cube_marker.pose.position.y = bb_center_worldframe.y();
        cube_marker.pose.position.z = bb_center_worldframe.z();
        cube_marker.pose.orientation.x = bb_orientation.x();
        cube_marker.pose.orientation.y = bb_orientation.y();
        cube_marker.pose.orientation.z = bb_orientation.z();
        cube_marker.pose.orientation.w = bb_orientation.w();
        cube_marker.scale.x = bb_max.x - bb_min.x;
        cube_marker.scale.y = bb_max.y - bb_min.y;
        cube_marker.scale.z = bb_max.z - bb_min.z;
        cube_marker.color.r = 0;
        cube_marker.color.g = 1;
        cube_marker.color.b = 0;
        cube_marker.color.a = 0.5;

//        ROS_DEBUG_STREAM("Message: \n" << cube_marker);

        markers.markers.push_back(cube_marker);
#endif
    }

    marker_pub_.publish(markers);

}

bool table_detector::findTable(geometry_msgs::Point &detected_pt) {
    ROS_INFO_ONCE("findTable called");
    ROS_DEBUG_ONCE("findTable called");
    return false; // TODO
}

table_detector::~table_detector()
{
    pcl_sub_.shutdown();
}
