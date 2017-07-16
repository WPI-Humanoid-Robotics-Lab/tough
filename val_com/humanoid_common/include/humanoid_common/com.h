#pragma once

#include <string>
#include <map>
#include <exception>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>

#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>


class com{
private:
    // node handle
    ros::NodeHandle nh_;

    KDL::Tree kdl_tree_;
    std::map<std::string, robot_state_publisher::SegmentPair> segments_;
    std::string root_link_name_;
    std::string rfoot_link_name_;
    std::string lfoot_link_name_;
    // flag to enable or disable visualisation
    bool enbaleVisFlag_;

    KDL::Chain kdl_chain_right_;
    KDL::Chain kdl_chain_left_;

    // vis marker array
    visualization_msgs::MarkerArray com_vis_markers_;

    // vis subscriber
    ros::Publisher visualization_pub_;

    void init(void);
    bool getKDLTree(void);
    //void getCOMForLink(std::string link_name, KDL::Tree tree, KDL::ChainFkSolverPos_recursive *tree_fk_solver, float *com_x, float *com_y, float *com_z, float *mass);

    void addChildren(const KDL::SegmentMap::const_iterator segment);
    void createCOMMarker(const std::string& ns, const std::string& frame_id, double radius, const KDL::Vector& cog, visualization_msgs::Marker& marker);
    void getCOMForLink(const KDL::SegmentMap::const_iterator& current_seg, const std::map<std::string, double>& joint_positions,
                                     const KDL::Frame& tf, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot, double& m, KDL::Vector& com);

public:

    std::string robot_desc_string_;

    com (ros::NodeHandle& nh, std::string root_link_name="pelvis", std::string rfoot_link_name="rightFoot", std::string lfoot_link_name="leftFoot", bool enableVis=false);
    ~com();

   // void computeCOM(void);
   void computeCOM(const std::map<std::string, double>& joint_positions, tf::Point& COM, double& mass,
                                tf::Transform& tf_right_foot, tf::Transform& tf_left_foot);

   void publishCOMVisMarkers(void);
   bool isEnableVis() const;
   void setEnbaleVisFlag(bool enbaleVisFlag);
};
