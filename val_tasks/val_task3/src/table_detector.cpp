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

    ROS_DEBUG_STREAM("Got " << candidates->size() << " candidate planes");

    // Use RANSAC on each candidate with PERPENDICULAR_PLANE w/r/t z-axis to get surface height
    pcl::SACSegmentation<table_detector::Point> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis({0.0, 0.0, 1.0});
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);

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

        // chull gives implicitly closed shapes, this explicitly closes it
        cloud_hull->push_back(cloud_hull->front());

        // Calculate the lines
        using HullLine = std::tuple<const Eigen::Vector4f &/* point */, const Eigen::Vector4f & /* direction */, double>;
        std::vector<HullLine> hull_lines;
        std::transform(cloud_hull->begin(), cloud_hull->end() - 1, cloud_hull->begin() + 1, std::back_inserter(hull_lines),
            [](const table_detector::Point &pt1, const table_detector::Point &pt2) {
                const Eigen::Vector4f v1 = pt1.getVector4fMap(); // I think assigning will copy?
                const Eigen::Vector4f v2 = pt2.getVector4fMap();
                const Eigen::Vector4f dir = v2 - v1;
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
        const auto dist_threshold_sqr = std::pow(0.05 + (max_pt - centroid).norm(), 2);
        for (const auto &pt : legs_projected->points) {
            // fast filter by distance to centroid
//            if ((pt.getVector4fMap() - centroid).squaredNorm() > dist_threshold_sqr) continue;

            const auto closest_line = std::min_element(hull_lines.begin(), hull_lines.end(),
                 [&pt](const HullLine &a, const HullLine &b) {
                      return pcl::sqrPointToLineDistance(pt.getVector4fMap(), std::get<0>(a), std::get<1>(a), std::get<2>(a)) <
                             pcl::sqrPointToLineDistance(pt.getVector4fMap(), std::get<0>(b), std::get<1>(b), std::get<2>(b));
                 });

            // too lazy to figure out how to not compute this twice
//            if (pcl::sqrPointToLineDistance(pt.getVector4fMap(), std::get<0>(*closest_line), std::get<1>(*closest_line),
//                                            std::get<2>(*closest_line)) > 0.0025) continue;

            const Eigen::Vector3f &line_origin = std::get<0>(*closest_line).head<3>();
            const Eigen::Vector3f &line_dir = std::get<1>(*closest_line).head<3>().normalized();
            const double &line_length = std::sqrt(std::get<2>(*closest_line));
            const double &scalar_projection = (pt.getVector3fMap() - line_origin).dot(line_dir);
            const auto constrained_scalar_proj = std::max(0.0, std::min(line_length, scalar_projection));
            const auto proj_point = line_origin + scalar_projection * line_dir;

            leg_candidate_pts->points.emplace_back();
            leg_candidate_pts->points.back().x = proj_point(0);
            leg_candidate_pts->points.back().y = proj_point(1);
            leg_candidate_pts->points.back().z = proj_point(2);
        }


        leg_candidate_pts->header = cloud->header;
        points_pub_.publish(leg_candidate_pts);


        const double area = chull.getTotalArea();

        const auto alpha = 1 - std::abs(1.71 - area) / 3;
        const auto rendered_alpha = std::min(1., std::max(0., 0.5 * alpha));

#if !DISABLE_DRAWINGS
        auto marker = visualization_msgs::Marker();
        pcl_conversions::fromPCL(cloud->header, marker.header);
        marker.ns = "table_candidates";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.01; // segment width in m
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = rendered_alpha;

        for (const auto &pt : cloud_hull->points) {
            marker.points.emplace_back();
            marker.points.back().x = pt.x;
            marker.points.back().y = pt.y;
            marker.points.back().z = pt.z;
        }

        // LINE_STRIP doesn't automatically close
        marker.points.emplace_back();
        marker.points.back().x = cloud_hull->points.front().x;
        marker.points.back().y = cloud_hull->points.front().y;
        marker.points.back().z = cloud_hull->points.front().z;

        markers.markers.push_back(marker);

        ROS_DEBUG_STREAM("Added marker with opacity " << alpha << " (rendered " << rendered_alpha << ")");
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
