#include "val_task3/rotate_valve.h"

rotateValve::rotateValve(ros::NodeHandle n):nh_(n), armTraj_(nh_), gripper_(nh_), walk_(nh_)
{
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);

    /* Top Grip Flat Hand modified*/
    leftHandOrientationTop_.header.frame_id         = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientationTop_.quaternion.x            = 0.604;
    leftHandOrientationTop_.quaternion.y            = 0.434;
    leftHandOrientationTop_.quaternion.z            = -0.583;
    leftHandOrientationTop_.quaternion.w            = 0.326;

    leftHandOrientationSide_.header.frame_id        = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientationSide_.quaternion.x           = 0.155;
    leftHandOrientationSide_.quaternion.y           = -0.061;
    leftHandOrientationSide_.quaternion.z           = -0.696;
    leftHandOrientationSide_.quaternion.w           = 0.699;

    leftHandOrientationSideUp_.header.frame_id      = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientationSideUp_.quaternion.x         = 0.211;
    leftHandOrientationSideUp_.quaternion.y         = 0.248;
    leftHandOrientationSideUp_.quaternion.z         = -0.644;
    leftHandOrientationSideUp_.quaternion.w         = 0.692;

    leftHandOrientationSideDown_.header.frame_id    = VAL_COMMON_NAMES::PELVIS_TF;
    leftHandOrientationSideDown_.quaternion.x       = -0.341;
    leftHandOrientationSideDown_.quaternion.y       = -0.312;
    leftHandOrientationSideDown_.quaternion.z       = -0.655;
    leftHandOrientationSideDown_.quaternion.w       =  0.598;


    // cartesian planners for the arm
    left_arm_planner_       = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    wholebody_controller_   = new wholebodyManipulation(nh_);
    chest_controller_       = new chestTrajectory(nh_);

    marker_pub               = nh_.advertise<visualization_msgs::MarkerArray>( "valve_path", 10, true);
}

bool rotateValve::grab_valve(const geometry_msgs::Point &goal, float executionTime)
{
    // set initial gripper pose with index fingers rounded
    // move to seed point somewhere above the valve
    // move to goal point and grasp the valve

    geometry_msgs::Point grabGoal;
    ROS_INFO("rotateValve: Setting gripper position");

    //Converting the centre to the grasp point based on the dimensions calculated manually
    current_state_->transformPoint(goal,grabGoal,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
    grabGoal.y -= 0.05241;
    grabGoal.z += 0.1601;
    current_state_->transformPoint(grabGoal,grabGoal,VAL_COMMON_NAMES::PELVIS_TF);

    std::vector<double> gripper1,gripper2,gripper3;

    gripper1 = {1.2, -1.35, -0.1, -0.1 ,-0.1 };
    gripper2 = {1.35, 0.0, -0.40, -0.40 ,-0.40 };

    gripper_.controlGripper(LEFT,gripper1);
    ros::Duration(executionTime/2).sleep();

    gripper_.controlGripper(LEFT,gripper2);
    ros::Duration(executionTime/2).sleep();

    ROS_INFO("rotateValve: Setting left arm seed position");

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(LEFT_SHOULDER_SEED_INITIAL);

    armTraj_.moveArmJoints(LEFT, armData, executionTime);

    ros::Duration(executionTime).sleep();

    geometry_msgs::QuaternionStamped temp  =leftHandOrientationTop_;
    current_state_->transformQuaternion(temp,temp);

    geometry_msgs::Point intermGoal;

    current_state_->transformPoint(grabGoal,intermGoal, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);

    intermGoal.x += 0.03;

    current_state_->transformPoint(intermGoal,intermGoal, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);

    geometry_msgs::Pose final;
    std::vector<geometry_msgs::Pose> waypoints;

    final.position=intermGoal;
    final.position.z+=0.08;
    final.orientation=temp.quaternion;


    waypoints.push_back(final);
    final.position=intermGoal;
    final.orientation=temp.quaternion;
    waypoints.push_back(final);

    moveit_msgs::RobotTrajectory traj;

    // Sending waypoints to the planner
    left_arm_planner_->getTrajFromCartPoints(waypoints,traj,false);

    // Planning whole body motion
    wholebody_controller_->compileMsg(LEFT,traj.joint_trajectory);
    ros::Duration(executionTime*1.5).sleep();

    // Grasping the cable
    ROS_INFO("rotateValve:Not Closing the gripper ");
    //gripper3={1.35, -0.60, -0.50, -0.50 ,-0.50 };
    //gripper_.controlGripper(LEFT,gripper3);
   // ros::Duration(executionTime/2).sleep();
    //gripper_.closeGripper(LEFT);
    //ros::Duration(executionTime/2).sleep();

    return true;


}

bool rotateValve::compute_traj(geometry_msgs::Point center, float radius, std::vector<geometry_msgs::Pose> &points)
{
    // using center point and radius, find points of the circle
    // plot using markers

    int NumSteps=21;
    geometry_msgs::Point centerPelvis;
    current_state_->transformPoint(center,centerPelvis, VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF);
    geometry_msgs::Pose point;
    for (int i = 0; i < (NumSteps-5)/2-1; ++i) {
        point.position.x=centerPelvis.x;
        point.position.y=centerPelvis.y + radius*cos((i*2*M_PI/NumSteps)-(M_PI/2));
        point.position.z=centerPelvis.z + radius*sin((i*2*M_PI/NumSteps)-(M_PI/2));


//    int NumSteps = 18;
//    for(int i = 0; i<(NumSteps/2-1);++i){
//        point.positsion.x=centerPelvis.x;
//        point.position.y=centerPelvis.y + radius*cos(i*2*M_PI/NumSteps);
//        point.position.z=centerPelvis.z + radius*sin(i*2*M_PI/NumSteps);


        if(i<=1)
        {
            point.orientation=leftHandOrientationSide_.quaternion;
        }
        else if(i>1 && i<=3)
        {
            point.orientation=leftHandOrientationSideUp_.quaternion;
        }
        else if(i>3 && i<=6)
        {
            point.orientation=leftHandOrientationTop_.quaternion;
        }
        current_state_->transformPose(point,point, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
        points.push_back(point);
    }
    visualise_traj(points);
    return true;

}

bool rotateValve::visualise_traj(std::vector<geometry_msgs::Pose> &points)
{
    visualization_msgs::MarkerArray markers = visualization_msgs::MarkerArray();
    visualization_msgs::Marker marker;

    for (int i = 0; i < points.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        marker.header.stamp = ros::Time();
        marker.ns = "valve-path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = points[i];
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 0.6;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0);
        markers.markers.push_back(marker);
    }

    marker_pub.publish( markers );
    ros::Duration(0.2).sleep();
    ROS_INFO("rotateValve: published trajectory markers");

}

bool rotateValve::move_valve(std::vector<geometry_msgs::Pose> poses, float executionTime)
{
    // using whole body controller follow the seed points
    // leave grasp
    // orient chest back to (0,0,0)

    std::vector<geometry_msgs::Pose> waypoints;
    for (int i = 0; i < poses.size(); ++i) {
        waypoints.push_back(poses[poses.size()-i-1]);
    }

    moveit_msgs::RobotTrajectory traj;
    left_arm_planner_->getTrajFromCartPoints(waypoints,traj,false);

    // Planning whole body motion
    wholebody_controller_->compileMsg(LEFT,traj.joint_trajectory);
    ros::Duration(executionTime*4.0).sleep();

    ROS_INFO("rotateValve: Done executing motion !!!");

    ROS_INFO("rotateValve: Opening Grippers");
    gripper_.openGripper(LEFT);
    ros::Duration(executionTime/2).sleep();

    std::vector< std::vector<float> > armData;
    //Moving down and towards the robot
    armData.clear();
    armData.push_back({-0.23,-1.21,0.65,-0.84,1.28,0,0});
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    armData.clear();
    armData.push_back({-0.23,-0.74,0.65,-0.84,1.28,0,0});
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    //Moving arm outside
    armData.clear();
    armData.push_back({0.1,0.1,0.1,0.1,0.1,0.1,0.1});
    armTraj_.moveArmJoints(LEFT, armData, executionTime);
    ros::Duration(executionTime).sleep();

    ROS_INFO("rotateValve: Adjusting chest to zero position");
    chest_controller_->controlChest(0,0,0);
    ros::Duration(executionTime).sleep();
}

bool rotateValve::reOrientbeforgrab(geometry_msgs::Point valveCenter)
{
    //Alligning relative to the centre of the valve
    geometry_msgs::Pose   pelvisPose;
    geometry_msgs::Pose2D preDoorOpenGoal;

    current_state_->transformPoint(valveCenter,valveCenter,
                                   VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
    current_state_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);

    valveCenter.x  -= 0.5;
    valveCenter.y  -= 0.45;
    //Converting back to world
    current_state_->transformPoint(valveCenter, valveCenter, VAL_COMMON_NAMES::PELVIS_TF);

    preDoorOpenGoal.x        = valveCenter.x;
    preDoorOpenGoal.y        = valveCenter.y;
    preDoorOpenGoal.theta    = tf::getYaw(pelvisPose.orientation);


    bool result = walk_.walkToGoal(preDoorOpenGoal);
    ROS_INFO("retOrientbeforegrab: the walking controller returned: %d",result);
    ros::Duration(1.0).sleep();
    return result;
}

rotateValve::~rotateValve(){

    if (left_arm_planner_!= nullptr)        delete left_arm_planner_;
    if (wholebody_controller_!= nullptr)    delete wholebody_controller_;
    if (chest_controller_!= nullptr)        delete chest_controller_;

}
