#include <val_moveit_planners/val_cartesian_planner.h>

cartesianPlanner::cartesianPlanner(std::string group_name, std::string reference_frame):
    group_name_(group_name), reference_frame_(reference_frame)

{
    /**************************************
     * ALWAYS START SPINNER IF USING MOVEIT
     *************************************/
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // set the planning group
    group_ = new moveit::planning_interface::MoveGroup(group_name_);
}

cartesianPlanner::~cartesianPlanner(){

    // delete the pointer
    if(group_ != nullptr)    delete group_;
}

// cartesian planner
double cartesianPlanner::getTrajFromCartPoints(std::vector<geometry_msgs::Pose> &points, moveit_msgs::RobotTrajectory &trajectory, bool avoid_collisions)
{
    // set the start state to the current state of the robot
    group_->setStartStateToCurrentState();

    // get the current joints and their positions
    std::vector<std::string> joint_names;
    joint_names = group_->getActiveJoints();
    std::vector<double> joint_values;
    joint_values = group_->getCurrentJointValues();
    std::vector<std::string>::iterator it;
    std::vector<double>::iterator itd;
//    ROS_INFO("current state");
//    for (it=joint_names.begin(), itd=joint_values.begin(); it<joint_names.end(); it++, itd++){
//        std::cout << *it << ": " << *itd << std::endl;
//    }

//    ROS_INFO("Reference frame: %s", group_->getPlanningFrame().c_str());
//    ROS_INFO("Reference frame: %s", group_->getEndEffectorLink().c_str());

    /**********************************************************************
       * These Parameters will alter the behaviour significantly
       *
       * RTT gives promising results
       * Planning attempts should be one
       * Planning time depends on the gola location (7 should be enough)
       * Goal tolerance is required as we dont have a defined orientation
       **********************************************************************/
    group_->setPlannerId("RRTkConfigDefault");
    /*
     * Other planners to experiment
    - SBLkConfigDefault
    - ESTkConfigDefault
    - LBKPIECEkConfigDefault
    - BKPIECEkConfigDefault
    - KPIECEkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - TRRTkConfigDefault
    - PRMkConfigDefault
    - PRMstarkConfigDefault
     */

    group_->setNumPlanningAttempts(1);
    group_->setPlanningTime(30);
    group_->setGoalTolerance(0.1);




    // compute the trajectory
    ROS_INFO("Planning cartesian path now");
    group_->setPoseReferenceFrame(reference_frame_);
    double frac = 0.0;
    int retry = 0;
    while (frac < 0.98 && retry++ < 5){
        frac = group_->computeCartesianPath(points, 0.01, 0.0, trajectory, avoid_collisions);
    }

    std::cout<<"Fraction of path planned:   "<<frac*100<<" % \n";
//    ROS_INFO("joint names");
//    for (it = trajectory.joint_trajectory.joint_names.begin(); it < trajectory.joint_trajectory.joint_names.end(); it++)
//        std::cout << *it << std::endl;
//    std::cout << "joint trajectories" << std::endl;
//    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator itp = trajectory.joint_trajectory.points.begin(); itp < trajectory.joint_trajectory.points.end(); itp++)
//        std::cout << *itp << std::endl;

    // return the fraction of path planned
    return frac;
}
