# pragma once

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>

class ValManipulation {

private:

    // node handle
    ros::NodeHandle nh_;

public:

    ValManipulation(ros::NodeHandle& nh);
    ~ValManipulation();

    void solve_ik(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param);
    double fRand(double min, double max);
};
