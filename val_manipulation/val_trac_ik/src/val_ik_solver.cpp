#include <val_manipulation/val_manipulation.h>

int main(int argc, char** argv)
{
    srand(1);
    ros::init(argc, argv, "val_track_ik");
    ros::NodeHandle nh("~");
    ValManipulation ik_solver(nh);
    tf::StampedTransform transform;
    tf::TransformListener listener;

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
    while(ros::ok()) {
        try {
            listener.waitForTransform("/leftPelvis", "/buttonFrame",ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/leftPelvis", "/buttonFrame", ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
        }
    }



    ik_solver.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, transform);

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
