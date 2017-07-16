#include <humanoid_common/com.h>

//construct KDL tree from urdf

com::com(ros::NodeHandle &nh, std::string root_link_name, std::string rfoot_link_name, std::string lfoot_link_name, bool enableVis):
    nh_(nh),root_link_name_(root_link_name), rfoot_link_name_(rfoot_link_name), lfoot_link_name_(lfoot_link_name)
{
    // !!!!!!!!!!!!! FIX THIS
    enbaleVisFlag_=true; //enableVis;
    // initialise
    init();
}

com::~com()
{

}

// initialise the required stuff

void com::init(void)
{
    nh_.getParam("/robot_description", robot_desc_string_);

    if (!kdl_parser::treeFromString(robot_desc_string_, kdl_tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    else
    {
        //get the KDL tree and coompute the branches
        if(!getKDLTree())
        {
            ROS_ERROR("KDL tree parsing failed, bummer BULLSHITTTTT..!!!!!!!!!!!");
        }
    }

    // for visualisation
    visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("stability_vis", 1);

}

// get the kdl tree from the robot description
bool com::getKDLTree(void)
{
    // walk the tree and add segments to segments_
    addChildren(kdl_tree_.getRootSegment());

    if (!(kdl_tree_.getChain(root_link_name_, rfoot_link_name_, kdl_chain_right_)
          && kdl_tree_.getChain(root_link_name_, lfoot_link_name_, kdl_chain_left_))) {
        ROS_ERROR("Could not initialize leg chains");
        return false;
    }

    ROS_INFO("kdl tree fecthed and parsed sucessfully");
    return true;
}

/////////////////////////  COM computation, most reference from hrl_kinematics ///////////////////////////////////////

//// add children to the tree
void com::addChildren(const KDL::SegmentMap::const_iterator segment)
{
    const std::string& root = segment->second.segment.getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
    for (unsigned int i=0; i<children.size(); i++){
        const KDL::Segment& child = children[i]->second.segment;
        robot_state_publisher::SegmentPair s(children[i]->second.segment, root, child.getName());
        if (child.getJoint().getType() == KDL::Joint::None){
            ROS_INFO("Tree initialization: Skipping fixed segment from %s to %s", root.c_str(), child.getName().c_str());
        }
        else{
            segments_.insert(make_pair(child.getJoint().getName(), s));
            ROS_INFO("Tree initialization: Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
        }
        addChildren(children[i]);
    }
}


void com::getCOMForLink(const KDL::SegmentMap::const_iterator& current_seg, const std::map<std::string, double>& joint_positions,
                        const KDL::Frame& tf, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot, double& m, KDL::Vector& com) {

    double jnt_p = 0.0;

    if (current_seg->second.segment.getJoint().getType() != KDL::Joint::None){
        std::map<std::string, double>::const_iterator jnt = joint_positions.find(current_seg->second.segment.getJoint().getName());

        if (jnt == joint_positions.end()){
            ROS_WARN("Could not find joint %s of %s in joint positions. Aborting tree branch.", current_seg->second.segment.getJoint().getName().c_str(), current_seg->first.c_str());
            return;
        }
        jnt_p = jnt->second;
    }

    KDL::Frame current_frame = tf * current_seg->second.segment.pose(jnt_p);
    //std::cout << current_seg->first << std::endl;
    if (current_seg->first == lfoot_link_name_){
        tf_left_foot = current_frame;
       // ROS_INFO("Right foot tip transform found");
    } else if (current_seg->first == rfoot_link_name_){
        tf_right_foot = current_frame;
       // ROS_INFO("Left foot tip transform found");
    }


    KDL::Vector current_cog = current_seg->second.segment.getInertia().getCOG();
    double current_m = current_seg->second.segment.getInertia().getMass();


    com = com + current_m * (current_frame*current_cog);

    m += current_m;
   // ROS_INFO("At link %s. local: %f / [%f %f %f]; global: %f / [%f %f %f]",current_seg->first.c_str(), current_m, current_cog.x(), current_cog.y(), current_cog.z(),
   //          m, com.x(), com.y(), com.z());

    // For visualisation, if requested
    if (isEnableVis())
    {
        if (current_m > 0.0){
            visualization_msgs::Marker marker;
            createCOMMarker(current_seg->first, "pelvis", 0.02, (current_frame*current_cog), marker);
            //std::cout << marker.pose.position.x << ' ' << marker.pose.position.y << ' ' << marker.pose.position.z << ' ' <<com_vis_markers_.markers.size() << std::endl;
            com_vis_markers_.markers.push_back(marker);
        }
    }

    std::vector<KDL::SegmentMap::const_iterator >::const_iterator child_it;
    for (child_it = current_seg->second.children.begin(); child_it !=current_seg->second.children.end(); ++child_it){
        getCOMForLink(*child_it, joint_positions, current_frame, tf_right_foot, tf_left_foot, m, com);
    }

}

void com::computeCOM(const std::map<std::string, double>& joint_positions, tf::Point& COM, double& mass,
                     tf::Transform& tf_right_foot, tf::Transform& tf_left_foot){
    mass = 0.0;
    KDL::Vector com;
    KDL::Frame ident = KDL::Frame::Identity();
    KDL::Frame transform = ident;
    KDL::Frame right_foot_tf = ident;
    KDL::Frame left_foot_tf = ident;

    // publish the link com as rviz markers, if requested
    if (isEnableVis())
    {
        publishCOMVisMarkers();
    }

    getCOMForLink(kdl_tree_.getRootSegment(), joint_positions, transform, right_foot_tf, left_foot_tf, mass, com);
    // TODO: fix (or)
    if (left_foot_tf == ident && right_foot_tf == ident){
        ROS_ERROR("Could not obtain foot transforms");
        return;
    }

    if (mass <= 0.0){
        ROS_ERROR("Total mass is 0, no CoM possible.");
        COM.setValue(0,0,0);
        return;
    }

    com = 1.0/mass * com;
   // ROS_INFO("Total mass: %f CoG: (%f %f %f)", mass, com.x(), com.y(), com.z());

    COM.setValue(com.x(), com.y(), com.z());
    tf::transformKDLToTF(right_foot_tf, tf_right_foot);
    tf::transformKDLToTF(left_foot_tf, tf_left_foot);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// visualisation functions

bool com::isEnableVis() const
{
    return enbaleVisFlag_;
}

void com::setEnbaleVisFlag(bool enbaleVisFlag)
{
    enbaleVisFlag_ = enbaleVisFlag;
}

void com::createCOMMarker(const std::string& ns, const std::string& frame_id, double radius, const KDL::Vector& cog, visualization_msgs::Marker& marker) {
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = cog.x();
    marker.pose.position.y = cog.y();
    marker.pose.position.z = cog.z();
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.b = 0.3;
    marker.color.r = 1.0;
    marker.color.a = 0.7;
}

void com::publishCOMVisMarkers(void){

    // publish the markers and reset the array
    visualization_pub_.publish(com_vis_markers_);

    // reset the viz markers
    com_vis_markers_.markers.clear();
}

/////////////////////////////////////////  Another approach, entirely relying on kd tree /////////////////////////////////////////////////////////
#if(0)
void humanoidCommon::computeCOM(void) {

    if (getKDLTree())
    {
        float com_x, com_y, com_z, mass;
        uint njoints = my_tree_.getNrOfJoints();
        KDL::ChainFkSolverPos_recursive tree_fk_solver = KDL::ChainFkSolverPos_recursive(my_tree_);

        // get the segment map
        KDL::SegmentMap segment_map = my_tree_.getSegments();
        KDL::SegmentMap::const_iterator it;

        int i = 1;
        std::cout << "joints = " << njoints << std::endl;
        std::cout << "segments = " << my_tree_.getNrOfSegments() << std::endl;

        for (it=segment_map.begin(); it!=segment_map.end();it++)
        {
            std::cout << i << " " << it->first << ' ' << it->second.segment.getInertia().getCOG()[0] \
                      << ' ' << it->second.segment.getInertia().getCOG()[1] \
                      << ' ' << it->second.segment.getInertia().getCOG()[2] \
                      << ' ' << it->second.segment.getInertia().getMass()<< std::endl;
            i = i+1;

            // compute the com for each link
            getCOMForLink(it->first, my_tree_, &tree_fk_solver, &com_x, &com_y, &com_z);
        }

    }
    else
    {

    }
}


void humanoidCommon::getCOMForLink(std::string link_name, KDL::Tree tree, KDL::ChainFkSolverPos_recursive *tree_fk_solver, float *com_x, float *com_y, float *com_z, float *mass)
{

    KDL::Frame tree_cartpos_frame;

    // get the number of joints
    int num_joints = tree.getNrOfJoints();
    KDL::JntArray tjointpositions = KDL::JntArray(num_joints);

    // fetch the joint positions


    // Calculate forward position kinematics
    bool kin_flag;
    kin_flag = tree_fk_solver->JntToCart(*tjointpositions,tree_cartpos_frame, link_name);

    // std::map<std::string,KDL::TreeElement>::const_iterator s1 = tree.getSegment(link_name);
    KDL::RigidBodyInertia inertia;
    inertia = tree.getSegment(link_name)->second.segment.getInertia();

    *mass = *mass + inertia.getMass();

    KDL::Vector v1 = inertia.getCOG();
    KDL::Vector v2 = tree_cartpos_frame * v1;

    *com_x = *com_x + v2.x() * inertia.getMass();
    *com_y = *com_y + v2.y() * inertia.getMass();
    *com_z = *com_z + v2.z() * inertia.getMass();
}

#endif
