#include <humanoid_common/standing_stable_controller.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <chrono>


stableController::stableController(ros::NodeHandle nh, bool vis_flag):
    nh_(nh), vis_flag_(vis_flag)
{
    COM_ = new com(nh, "rightAnklePitchLink", "rightFoot", "leftFoot", true);
    stability_ = new stability(1, nh, "rightFoot", "rightAnklePitchLink", "rightFoot", "leftFoot");
    joint_state_ = new valJointState(nh);

    // control loop frequency
    dt_ = 0.01; //10ms

    if (vis_flag_ == true)
    {
        support_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("support_polygon", 1);
        pcom_pub_ = nh_.advertise<visualization_msgs::Marker>("projected_COM", 1);
        act_com_marker_ = nh_.advertise<visualization_msgs::Marker>("COM_wrt_root", 1);
    }

    right_effort_ = nh_.advertise<std_msgs::Float64>("rightAnklePitch_effort_controller/command", 1);
    left_effort_ = nh_.advertise<std_msgs::Float64>("leftAnklePitch_effort_controller/command", 1);

    rightHipYaw_pub_ = nh_.advertise<std_msgs::Float64>("rightHipYaw_effort_controller/command", 1);
    rightHipRoll_pub_ = nh_.advertise<std_msgs::Float64>("rightHipRoll_effort_controller/command", 1);
    rightHipPitch_pub_ = nh_.advertise<std_msgs::Float64>("rightHipPitch_effort_controller/command", 1);
    rightKneePitch_pub_ = nh_.advertise<std_msgs::Float64>("rightKneePitch_effort_controller/command", 1);
    rightAnklePitch_pub_ = nh_.advertise<std_msgs::Float64>("rightAnklePitch_effort_controller/command", 1);
    rightAnkleRoll_pub_ = nh_.advertise<std_msgs::Float64>("rightAnkleRoll_effort_controller/command", 1);

    // set the equilibrium com wrt pelvis
    // 0.094,0.102,-0.24
    com_desired_wrt_root_.setX(0.094);
    com_desired_wrt_root_.setY(0.102);
    com_desired_wrt_root_.setZ(0.24);

    // create a file
    file_.open ("/home/sumanth/indigo_ws/src/valkyrie_control/plot.txt");
    // write the header
    file_ << "joint names: ";
    for(int i=0; i<chain_.getNrOfSegments(); i++)
    {
        file_ << chain_.getSegment(i).getJoint().getName() << "   ";
    }
    file_ << "\n";
    file_ << "com_x, com_y, com_z, jpos1, jpos2, jpos3, jpos4, jpos5, jpos6, \
             j1_vel, j2_vel, j3_vel, j4_vel, j5_vel, j6_vel, \
            j1_acc, j2_acc, j3_acc, j4_acc, j5_acc, j6_acc, \
            j1_T, j2_T, j3_T, j4_T, j5_T, j6_T \n";

            control_thread_ = std::thread(&stableController::controlLoop, this);
    control_thread_.detach();
}

stableController::~stableController()
{
    ROS_INFO("stable controller destructor called");
}


visualization_msgs::Marker stableController::getCOMMarkerwrtRoot(void)
{
    visualization_msgs::Marker com_marker;
    com_marker.header.stamp = ros::Time::now();
    com_marker.header.frame_id = "rightAnklePitchLink";
    com_marker.action = visualization_msgs::Marker::ADD;
    com_marker.type = visualization_msgs::Marker::SPHERE;
    com_marker.pose.position.x = com_wrt_root().getX();
    com_marker.pose.position.y = com_wrt_root().getY();
    com_marker.pose.position.z = com_wrt_root().getZ();
    com_marker.scale.x = 0.05;
    com_marker.scale.y = 0.05;
    com_marker.scale.z = 0.01;
    com_marker.color.a = 1.0;
    com_marker.color.g = 0.0;
    com_marker.color.r = 0.0;
    com_marker.color.b = 1.0;

    return com_marker;
}

void stableController::initSolvers(void)
{
    // solvers
    // create ik solver
    tracik_solver_ = new TRAC_IK::TRAC_IK("rightFoot", "rightHipYawLink", "/robot_description", 10, 0.00001);
    bool valid = tracik_solver_->getKDLChain(chain_);
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
    }

    //    std::cout << "segments " << chain_.getNrOfSegments() << " joints " << chain_.getNrOfJoints() <<std::endl;
    //    for (std::size_t i=0; i<chain_.getNrOfSegments(); i++)
    //    {
    //        std::cout<< "segment: " << chain_.getSegment(i).getName() << " joint " << chain_.getSegment(i).getJoint().getName() <<std::endl;
    //    }
    std::cout <<std::endl;
    chain_.addSegment(KDL::Segment("com_pelvis", KDL::Joint("rightHipYaw", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(com_wrt_root().getX(), com_wrt_root().getY(), com_wrt_root().getZ()))));

//        std::cout << "segments " << chain_.getNrOfSegments() << " joints " << chain_.getNrOfJoints() <<std::endl;
//        for (std::size_t i=0; i<chain_.getNrOfSegments(); i++)
//        {
//            std::cout<< "segment: " << chain_.getSegment(i).getName() << " joint " << chain_.getSegment(i).getJoint().getName() <<std::endl;
//        }

    jnt_jac_solver_ =  new KDL::ChainJntToJacSolver(chain_);
    KDL::Vector grav;
    grav.data[0] = 0.0;
    grav.data[1] = 0.0;
    grav.data[2] = -9.81;
    inv_dynamics_solver_ = new KDL::ChainIdSolver_RNE(chain_, grav);

    // resize
    static bool firstrun = true;
    if (firstrun)
    {
        ROS_INFO("resized");
        q_.resize(chain_.getNrOfJoints());
        q_dot_(chain_.getNrOfJoints(), 1);
        q_dot_dot_(chain_.getNrOfJoints(), 1);

        firstrun = false;
    }

}

void stableController::updateCOM(void)
{
    // clear the joint positions
    joint_positions_.clear();

    // get the joint states map
    for (std::size_t i=0; i<joint_state_->getJointStates().name.size(); i++)
    {
        joint_positions_.insert(make_pair(joint_state_->getJointStates().name[i], joint_state_->getJointStates().position[i]));
        //std::cout<< "name: " << joint_state_->getJointStates().name[i] << " pos: " << joint_state_->getJointStates().position[i]<<std::endl;
    }

    // transforms from root to left and right foot:
    tf::Transform tf_right_foot, tf_left_foot;
    COM_->computeCOM(joint_positions_, com_wrt_root_, mass_robot_, tf_right_foot, tf_left_foot);

    // is pose stable
    tf::Vector3 normal_vector(0.0, 0.0, 1.0); // planar support
    // for testing, normal vector of arbitrary plane:
    normal_vector.normalize();
    pose_stable_ = stability_->isPoseStable(joint_positions_, stability::FootSupport::SUPPORT_DOUBLE, normal_vector);

    ROS_INFO("mass: %f", mass_robot_);
    ROS_INFO("com wrt root: x %f, y: %f, z: %f",com_wrt_root_.getX(), com_wrt_root_.getY(), com_wrt_root_.getZ());
    ROS_INFO("projected com: x %f, y: %f, z: %f",stability_->getCOM().getX(), stability_->getCOM().getY(), stability_->getCOM().getZ());

    file_<< com_wrt_root_.getX() << "," << com_wrt_root_.getY() << "," << com_wrt_root_.getZ() << ",";

    // update the transformation between the pelvis and COM
    tf::TransformBroadcaster br;
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(com_wrt_root_.getX(), com_wrt_root_.getY(), com_wrt_root_.getZ()));
    // TODO: fix this, no orientation
    tf::Quaternion q;
    q.setRPY(0, 0, 1);
    tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "/rightAnklePitchLink", "/com_root"));

    if (vis_flag_ == true)
    {
        support_polygon_pub_.publish(stability_->getSupportPolygon());
        pcom_pub_.publish(stability_->getProjectedCOMMarker());
        act_com_marker_.publish(getCOMMarkerwrtRoot());
    }
}

void stableController::updateJntVelocities(void)
{

    for(int i=0; i<chain_.getNrOfSegments(); i++)
    {
        q_(i) = joint_positions_.find(chain_.getSegment(i).getJoint().getName())->second;
        //std::cout << "pos " << q_(i) << " joint name " << chain_.getSegment(i).getJoint().getName() << " segment name " << chain_.getSegment(i).getName() << " act pos "<< joint_positions_.find(chain_.getSegment(i).getJoint().getName())->second << std::endl;
        file_ << q_(i) << ",";
    }
    std::cout << std::endl;

    // compute the jacobian
    J_.resize(chain_.getNrOfJoints());
    //    for (std::size_t i=0; i<chain_.getNrOfJoints(); i++)
    //    {
    //        std::cout<< "segment: " << chain_.getSegment(i).getName() << " joint " << chain_.getSegment(i).getJoint().getName() <<std::endl;
    //    }
    //    std::cout << "before jac" << std::endl;
    //    for (int i=0; i<6; i++)
    //    {
    //        for(int j=0; j<5;j++)
    //        {
    //            std::cout << J_(i,j) << " ";
    //        }
    //        std::cout << std::endl;
    //    }
    jnt_jac_solver_->JntToJac(q_, J_);
    //    std::cout << "after jac" << std::endl;
    //    for (int i=0; i<6; i++)
    //    {
    //        for(int j=0; j<5;j++)
    //        {
    //            std::cout << J_(i,j) << " ";
    //        }
    //        std::cout << std::endl;
    //    }
}

void stableController::computeJointTorques(void)
{
    // use the jacobain to compute the desried joint angles
    Eigen::MatrixXf x_dot(chain_.getNrOfJoints(),1);
    x_dot(0,0) = (com_desired_wrt_root_.getX() - com_wrt_root().getX())/dt_;
    x_dot(1,0) = (com_desired_wrt_root_.getY() - com_wrt_root().getY())/dt_;
    x_dot(2,0) = (com_desired_wrt_root_.getZ() - com_wrt_root().getZ())/dt_;
    x_dot(3,0) = 0;
    x_dot(4,0) = 0;
    x_dot(5,0) = 0;

    // inverse kinematics valocity jacobain
    Eigen::MatrixXf jacobian_inv(6,chain_.getNrOfJoints());

    //std::cout << "Jacobian" << std::endl;
    for (int i=0; i<chain_.getNrOfJoints(); i++)
    {
        for(int j=0; j<chain_.getNrOfJoints();j++)
        {
            jacobian_inv(i,j) = J_(i,j);
            // std::cout << jacobian_inv(i,j) << " ";
        }
        //std::cout << std::endl;
    }

    jacobian_inv = jacobian_inv.inverse();
    //jacobian_inv = pseudoInverse(jacobian_inv);

    //    std::cout << "Inverse Jacobian" << std::endl;
    //    for (int i=0; i<chain_.getNrOfJoints(); i++)
    //    {
    //        for(int j=0; j<chain_.getNrOfJoints();j++)
    //        {
    //            std::cout << jacobian_inv(i,j) << " ";
    //        }
    //        std::cout << std::endl;
    //    }

    // joint velocities
    q_dot_ = jacobian_inv * x_dot;
    std::cout <<"vel: \n";
    for(int i = 0; i<chain_.getNrOfJoints(); i++)
    {
        file_ << q_dot_(i) << ",";
        std::cout << q_dot_(i) << " ";
        std::cout << std::endl;
    }

    static Eigen::MatrixXf q_dot_prev(6,1);

    // joint acclerations
    q_dot_dot_ = (q_dot_ - q_dot_prev)/dt_;
    std::cout <<"\naccel: \n";
    for(int i = 0; i<chain_.getNrOfJoints(); i++)
    {
        file_ << q_dot_dot_(i) << ",";
        std::cout << q_dot_dot_(i) << " ";
        std::cout << std::endl;
    }

    // update previous velocity
    q_dot_prev = q_dot_;

    // compute torques
    // using KDL which uses newton euler
    jntTorques_.resize(chain_.getNrOfJoints());
    KDL::Wrenches f_ext(chain_.getNrOfSegments());
    //f_ext(0) = 0.3; f_ext(1) = 0.3; f_ext(2) = 0.3; f_ext(3) = 0.3; f_ext(4) = 0.3;

    KDL::JntArray q_dot, q_dot_dot;
    q_dot.resize(chain_.getNrOfJoints());
    q_dot_dot.resize(chain_.getNrOfJoints());
    for(int i=0; i<chain_.getNrOfJoints();i++)
    {
        q_dot(i) = q_dot_(i,0);
        q_dot_dot(i) = q_dot_dot_(i,0);
    }

    // get the torques
    inv_dynamics_solver_->CartToJnt(q_, q_dot, q_dot_dot, f_ext, jntTorques_);

    std::cout << "Torques without cap" << std::endl;
    for(int i = 0; i<chain_.getNrOfJoints(); i++)
    {
        std::cout << jntTorques_(i) << " ";
    }
    std::cout << std::endl;
    // cap on tiorquw
    jntTorques_(5) = (fabs(jntTorques_(5))>1000.0 ? copysign(1000.0, ((jntTorques_(5) > 0) ? 1.0 : -1.0)) : jntTorques_(5));
    jntTorques_(4) = (fabs(jntTorques_(4))>1000.0 ? copysign(1000.0, ((jntTorques_(4) > 0) ? 1.0 : -1.0)) : jntTorques_(4));
    jntTorques_(3) = (fabs(jntTorques_(3))>1000.0 ? copysign(1000.0, ((jntTorques_(3) > 0) ? 1.0 : -1.0)) : jntTorques_(3));
    jntTorques_(2) = (fabs(jntTorques_(2))>1000.0 ? copysign(1000.0, ((jntTorques_(2) > 0) ? 1.0 : -1.0)) : jntTorques_(2));
    jntTorques_(1) = (fabs(jntTorques_(1))>1000.0 ? copysign(1000.0, ((jntTorques_(1) > 0) ? 1.0 : -1.0)) : jntTorques_(1));
    jntTorques_(0) = (fabs(jntTorques_(0))>1000.0 ? copysign(1000.0, ((jntTorques_(0) > 0) ? 1.0 : -1.0)) : jntTorques_(0));

    std::cout << "torques" <<std::endl;
    for (int i=0; i< chain_.getNrOfJoints(); i++)
    {
        std::cout << jntTorques_(i) << " ";
        file_ << jntTorques_(i) << ",";
    }
    std::cout<< std::endl;
    file_<<"\n";
}

void stableController::publishJoints(void)
{
    rightHipYaw_pub_.publish(jntTorques_(5));
    rightHipRoll_pub_.publish(jntTorques_(4));
    rightHipPitch_pub_.publish(jntTorques_(3));
    rightKneePitch_pub_.publish(jntTorques_(2));
    rightAnklePitch_pub_.publish(jntTorques_(1));
    rightAnkleRoll_pub_.publish(jntTorques_(0));
}

// control loop
void stableController::controlLoop(void)
{
    static int c =0;
    while(true)
    {
        // update the COM
        updateCOM();

        c++;

        if (c>10)
        {
            ROS_INFO("solver updated");
            // init the solvers
            initSolvers();

            // compute the jacobian
            updateJntVelocities();

            // compute the torques
            computeJointTorques();

            // publish joints
            publishJoints();

            c = 11;
        }

        // sleep
        sleep(dt_); // 10ms
    }
}


void stableController::getCOMtorques(void)
{
    double x = com_wrt_root().getX();
    double y = com_wrt_root().getY();
    double z = com_wrt_root().getZ();
    double m = mass_robot();
    double g = 9.8;

    double r = sqrt(x*x + y*y + z*z);

    double Sp = x/r;
    double Sr = -y/r;
    double D = z/r;

    double theta_p = asin(Sp);
    double theta_r = asin(Sr);

    double Cp = cos(theta_p);
    double Cr = cos(theta_r);

    double J[3][3] = {{0, r*Cp, Sp}, {-r*Cr, 0, -Sr}, {-r*Cr*Sr/D, -r*Cp*Sp/D, D}};

    // Add control inputs
    double up; //= kp_p;
    double ur; //= kp_r;

    double acc_x = (g*x/z) - (ur/(m*z));
    double acc_y = (g*y/z) - (up/(m*z));

    double torque_p = Cp*up/D;
    double torque_r = Cr*ur/D;

}


// setter getter functions
tf::Point stableController::com_desired_wrt_root() const
{
    return com_desired_wrt_root_;
}

void stableController::setCom_desired_wrt_root(const tf::Point &com_desired_wrt_pelvis)
{
    com_desired_wrt_root_ = com_desired_wrt_pelvis;
}

tf::Point stableController::com_wrt_root() const
{
    return com_wrt_root_;
}

void stableController::setCom_wrt_root(const tf::Point &com_wrt_pelvis)
{
    com_wrt_root_ = com_wrt_pelvis;
}

double stableController::mass_robot() const
{
    return mass_robot_;
}

void stableController::setMass_robot(double mass_robot)
{
    mass_robot_ = mass_robot;
}

bool stableController::is_pose_stable() const
{
    return pose_stable_;
}

void stableController::setPose_stable(bool pose_stable)
{
    pose_stable_ = pose_stable;
}

/************************************************************************************/
// Energy controllers functions for effort control of right and left ankle
std::vector<double> stableController::getThetas()
{
    // add a frame for the com
    tf::TransformBroadcaster br;
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(com_wrt_root().getX(), com_wrt_root().getY(), com_wrt_root().getZ()));
    tf::Quaternion q;
    q.setRPY(0, 0, 1);
    tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "/pelvis", "/COM_wrt_pelvis"));

    tf::TransformListener tf_listener;

    geometry_msgs::PointStamped rfoot_point, lfoot_point;
    rfoot_point.header.frame_id= "/rightFoot";
    rfoot_point.header.stamp = ros::Time(0);
    rfoot_point.point.x = com_wrt_root().getX();
    rfoot_point.point.y = com_wrt_root().getY();
    rfoot_point.point.z = com_wrt_root().getZ();
    geometry_msgs::PointStamped rfoot_point_transformed;

    lfoot_point.header.frame_id= "/leftFoot";
    lfoot_point.header.stamp = ros::Time(0);
    lfoot_point.point.x = com_wrt_root().getX();
    lfoot_point.point.y = com_wrt_root().getY();
    lfoot_point.point.z = com_wrt_root().getZ();
    geometry_msgs::PointStamped lfoot_point_transformed;

    try
    {
        tf_listener.waitForTransform("/pelvis", "/rightFoot", ros::Time(0), ros::Duration(30.0));
        tf_listener.transformPoint("/pelvis", rfoot_point, rfoot_point_transformed);
        tf_listener.waitForTransform("/pelvis", "/leftFoot", ros::Time(0), ros::Duration(30.0));
        tf_listener.transformPoint("/pelvis", lfoot_point, lfoot_point_transformed);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    std::cout << "original com: " << com_wrt_root().getX() << " " << com_wrt_root().getY() << " " << com_wrt_root().getZ() << std::endl;
    std::cout << "rtransformed com: " << rfoot_point_transformed.point.x << " " << rfoot_point_transformed.point.y << " " << rfoot_point_transformed.point.z << std::endl;
    std::cout << "ltransformed com: " << lfoot_point_transformed.point.x << " " << lfoot_point_transformed.point.y << " " << lfoot_point_transformed.point.z << std::endl;

    // transform the right foot to pelvis frame
    double rtheta = acos(rfoot_point_transformed.point.y/sqrt(rfoot_point_transformed.point.x*rfoot_point_transformed.point.x + rfoot_point_transformed.point.y*rfoot_point_transformed.point.y));
    double ltheta = acos(lfoot_point_transformed.point.y/sqrt(lfoot_point_transformed.point.x*lfoot_point_transformed.point.x + lfoot_point_transformed.point.y*lfoot_point_transformed.point.y));
    std::cout << "rangle: " << rtheta << std::endl;
    std::cout << "langle: " << ltheta << std::endl;

    std::vector<double> thetas = {rtheta, ltheta};

    return thetas;
}

void stableController::energyControl(double rtheta, double ltheta)
{
    if(rtheta > 0)
        r_theta_dot_ = -std::abs(rtheta - r_theta_prev_)/0.010;
    else
        r_theta_dot_ = std::abs(rtheta - r_theta_prev_)/0.010;

    if(ltheta > 0)
        l_theta_dot_ = -std::abs(ltheta - l_theta_prev_)/0.010;
    else
        l_theta_dot_ = std::abs(ltheta - l_theta_prev_)/0.010;

    rP_ = rtheta + sqrt(1.06/9.8)*r_theta_dot_;
    lP_ = ltheta + sqrt(1.06/9.8)*l_theta_dot_;
    std::cout<<"rP: " << rP_ << std::endl;
    std::cout<<"lP: " << lP_ << std::endl;
    r_control_input_ = -kp_*rP_;
    l_control_input_ = -kp_*lP_;

    // For full body
    //double r_torque_ = 136.078*9.8*1.06*sin(r_control_input_);
    //double l_torque_ = 136.078*9.8*1.06*sin(l_control_input_);

    // For lower body
    r_torque_ = 49.71*9.8*1.06*sin(r_control_input_);
    l_torque_ = 49.71*9.8*1.06*sin(l_control_input_);
    std::cout<<"r_torque: " << r_torque_ << std::endl;
    std::cout<<"l_torque: " << l_torque_ << std::endl;

    right_effort_.publish(r_torque_);
    left_effort_.publish(l_torque_);

    // Update prev theta
    r_theta_prev_ = rtheta;
    l_theta_prev_ = ltheta;
}
