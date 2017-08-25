# pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <humanoid_common/com.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>
#include <vector>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PolygonStamped.h>

class stability {

public:
    enum class FootSupport {SUPPORT_DOUBLE, SUPPORT_SINGLE_RIGHT, SUPPORT_SINGLE_LEFT};
    stability(float foot_polygon_scale, ros::NodeHandle nh, std::string foot_mesh_link, std::string root_link, std::string rfoot, std::string lfoot);
    ~stability();
    float getSupportPolygonScale() const;
    void setSupportPolygonScale(float foot_polygon_scale);
    tf::Point getCOM() const;
    bool isPoseStable(const std::map<std::string, double>& joint_positions, FootSupport support_mode);
    bool isPoseStable(const std::map<std::string, double>& joint_positions, FootSupport support_mode, const tf::Vector3& normal_vector);
    visualization_msgs::Marker getProjectedCOMMarker() const;
    geometry_msgs::PolygonStamped getSupportPolygon() const;

private:
    //Convex Hull scaling factor
    float support_polygon_scale_;
    boost::shared_ptr<const urdf::ModelInterface> urdf_model_;
    ros::NodeHandle nh_;
    std::string foot_mesh_link_, root_link_name_, rfoot_link_, lfoot_link_;
    com* CoM;

    // Support polygon (x,y) for the right foot
    std::vector<tf::Point> support_polygon_foot_right_;
    // Support polygon (x,y) for the left foot (mirrored from right)
    std::vector<tf::Point> support_polygon_foot_left_;
    std::vector<tf::Point> support_polygon_;
    tf::Transform tf_to_support_;
    tf::Point projected_COM_;

    void init(void);
    void initSupportPolygon(void);
    bool constructSupportPolygon(void);
    bool pointInConvexHull(const tf::Point& point, const std::vector<tf::Point>& polygon) const;
    std::vector<tf::Point> convexHull(const std::vector<tf::Point>& points) const;
};
