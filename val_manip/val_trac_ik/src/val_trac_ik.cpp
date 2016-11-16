#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>

double fRand(double min, double max)
{
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
}


void solve_ik(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{

    double eps = 1e-5;

    // This constructor parses the URDF loaded in rosparm urdf_param into the
    // needed KDL structures.  We then pull these out to compare against the KDL
    // IK solver.
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul; //lower joint limits, upper joint limits

    bool valid = tracik_solver.getKDLChain(chain);

    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }

    valid = tracik_solver.getKDLLimits(ll,ul);

    if (!valid) {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    ROS_INFO ("Using %d joints",chain.getNrOfJoints());


    // Set up KDL IK
    KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
    KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver
    // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)


    // Create Nominal chain configuration midway between all joint limits
    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j=0; j<nominal.data.size(); j++) {
        nominal(j) = (ll(j)+ul(j))/2.0;
    }

    //Create desired number of valid, random joint configurations
    std::vector<KDL::JntArray> JointList;
    KDL::JntArray q(chain.getNrOfJoints());

    for (uint i=0; i < num_samples; i++) {
        for (uint j=0; j<ll.data.size(); j++) {
            q(j)=fRand(ll(j), ul(j));
        }
        JointList.push_back(q);
    }


    //  boost::posix_time::ptime start_time;
    //  boost::posix_time::time_duration diff;

    KDL::JntArray result;

    KDL::Frame end_effector_pose;
    end_effector_pose.M = KDL::Rotation::Quaternion(0, 0, 0, 1.0);
    end_effector_pose.p = KDL::Vector(0.33, 0.45, 0.97);


    int rc;

    // double total_time=0;
    uint success=0;


    ROS_INFO_STREAM("*** Testing TRAC-IK with "<<num_samples<<" random samples");

   // for (uint i=0; i < num_samples; i++) {
       // fk_solver.JntToCart(JointList[i],end_effector_pose);
        //double elapsed = 0;
        //start_time = boost::posix_time::microsec_clock::local_time();
        rc=tracik_solver.CartToJnt(nominal,end_effector_pose,result);
        if (rc=0)
//            success++;
  //  }


    ROS_INFO_STREAM("TRAC-IK  "<< success ? "TRUE" : "FALSE");
    ROS_INFO_STREAM("Result data - " << result.data);
}

int main(int argc, char** argv)
{
    srand(1);
    ros::init(argc, argv, "val_track_ik");
    ros::NodeHandle nh("~");

    int num_samples;
    std::string chain_start, chain_end, urdf_param;
    double timeout;

    nh.param("num_samples", num_samples, 1);
    nh.param("chain_start", chain_start, std::string(""));
    nh.param("chain_end", chain_end, std::string(""));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    if (num_samples < 1)
        num_samples = 1;

    solve_ik(nh, num_samples, chain_start, chain_end, timeout, urdf_param);

    // Useful when you make a script that loops over multiple launch files that test different robot chains
    // std::vector<char *> commandVector;
    // commandVector.push_back((char*)"killall");
    // commandVector.push_back((char*)"-9");
    // commandVector.push_back((char*)"roslaunch");
    // commandVector.push_back(NULL);

    // char **command = &commandVector[0];
    // execvp(command[0],command);

    return 0;
}
