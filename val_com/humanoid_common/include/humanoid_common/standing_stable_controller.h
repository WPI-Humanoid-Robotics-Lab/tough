# pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <thread>
#include <humanoid_common/com.h>
#include <humanoid_common/stability.h>
#include <val_interface/val_JointState.h>
#include <vector>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <iostream>
#include <fstream>
#include<Eigen/Core>
#include<Eigen/SVD>

class stableController{
private:
    ros::NodeHandle nh_;

    bool vis_flag_;

    // Publishers for support polygon, projected com and actual com
    ros::Publisher support_polygon_pub_, pcom_pub_, act_com_marker_;

    // Publishers for right ankle and left ankle effort controllers
    ros::Publisher right_effort_, left_effort_;
    ros::Publisher rightHipYaw_pub_, rightHipRoll_pub_, rightHipPitch_pub_, rightKneePitch_pub_, rightAnklePitch_pub_, rightAnkleRoll_pub_;

    float dt_;

    // get the current joint positions
    KDL::JntArray  q_;

    // joint velocities and accelerations
    Eigen::MatrixXf q_dot_dot_, q_dot_;

    KDL::JntArray jntTorques_;

    // TODO: --> not good way to intialise variables here, should go to constructor
    // previous thetas and theta_dots for right and left ankle effort controllers
    double r_theta_prev_ = 0, l_theta_prev_ = 0, r_theta_dot_ = 0, l_theta_dot_ = 0;

    // variables for energy control
    double kp_ = 0.1, rP_, lP_, r_control_input_, l_control_input_, r_torque_, l_torque_;

    // joint positions
    std::map<std::string, double> joint_positions_;

    // object for com computation
    com* COM_;

    // object for stability
    stability* stability_;

    // object for val_interface
    valJointState* joint_state_;

    // mass of the robot
    double mass_robot_;

    //com with respect to pelvis
    tf::Point com_wrt_root_;
    tf::Point com_desired_wrt_root_;

    // jacobain
    KDL::Jacobian  J_;

    // is pose stable
    bool pose_stable_;

    // thread for controller
    std::thread control_thread_;

    KDL::Chain chain_;
    // solvers
    KDL::ChainJntToJacSolver* jnt_jac_solver_;
    KDL::ChainIdSolver_RNE* inv_dynamics_solver_;
    TRAC_IK::TRAC_IK* tracik_solver_;

    std::ofstream file_;

    visualization_msgs::Marker getCOMMarkerwrtRoot(void);
    void controlLoop (void);
    void updateJntVelocities(void);
    void updateCOM(void);
    void computeJointTorques(void);
    void initSolvers(void);
    void publishJoints(void);

  public:
    stableController(ros::NodeHandle nh, bool vis_flag=false);
    ~stableController();

    void jointStateCb(const sensor_msgs::JointStateConstPtr& state);    
    tf::Point com_wrt_root() const;
    void setCom_wrt_root(const tf::Point &com_wrt_root);
    double mass_robot() const;
    void setMass_robot(double mass_robot);
    bool is_pose_stable() const;
    void setPose_stable(bool pose_stable);
    std::vector<double> getThetas();
    void energyControl(double rtheta, double ltheta);
    void getCOMtorques(void);
    tf::Point com_desired_wrt_root() const;
    void setCom_desired_wrt_root(const tf::Point &com_desired_wrt_root);


    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
};
