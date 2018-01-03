#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_common/val_common_names.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <string>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <pthread.h>
#include "tough_common/robot_description.h"

using namespace visualization_msgs;
using namespace std;
ros::NodeHandle* nh_port;
ros::Publisher* pose_pub,*execute_pub;
RobotDescription* rd_;
interactive_markers::InteractiveMarkerServer* server_port;
ArmControlInterface *right_armTraj_port,*left_armTraj_port;
interactive_markers::MenuHandler menu_handler;
tf::TransformListener* listener_port;
geometry_msgs::PoseStamped right_pose,left_pose;
moveit::planning_interface::MoveGroup::Plan *right_plan,*left_plan;
bool planning = false,pose_received = false,execute = false,push_back = false;
bool left_pose_received = false, right_pose_received = false;
pthread_t input_thread;

void print_joints(moveit::planning_interface::MoveGroup* group)
{
    // get the cuurent joints and their positions
    std::vector<std::string> jNames;
    jNames = group->getActiveJoints();
    std::vector<double> jValues;
    jValues = group->getCurrentJointValues();
    std::vector<std::string>::iterator it;
    std::vector<double>::iterator itd;
    ROS_INFO("current state");
    for (it=jNames.begin(), itd=jValues.begin(); it<jNames.end(); it++, itd++){
        std::cout << *it << ": " << *itd << std::endl;
    }
}


int plan_trajectory(std::string MoveGroup_name,geometry_msgs::PoseStamped &input_pose,moveit::planning_interface::MoveGroup::Plan* ret_plan,string planner_type)
{
    moveit::planning_interface::MoveGroup* group;
    group = new moveit::planning_interface::MoveGroup(MoveGroup_name);
    group->setStartStateToCurrentState();
    print_joints(group);
    group->setPoseTarget(input_pose.pose);
    group->setPlannerId(planner_type);


    ROS_INFO("Begin Planning");
    bool success = group->plan((*ret_plan));
    if (success)
    {
        delete group;
        ROS_INFO("Successfully planned the trajectory");
        /* Sleep to give Rviz time to visualize the plan. */
        return 0;
    }
    else
    {
        delete group;
        ROS_WARN("planning failed");
        return -1;
    }

}

const geometry_msgs::PoseStamped transform_pose_simple(const geometry_msgs::PoseStamped *from_pose,std::string to_frame)
{
    geometry_msgs::PoseStamped input_pose;
    geometry_msgs::PoseStamped target_pose;
    input_pose.pose.position = from_pose->pose.position;
    input_pose.pose.orientation = from_pose->pose.orientation;
    input_pose.header = from_pose->header;

    tf::TransformListener listener;

    ros::Duration(0.2).sleep();
    try
    {
        ros::Time t = ros::Time::now();
        listener.waitForTransform(input_pose.header.frame_id, to_frame, t, ros::Duration(3.0));
        listener.transformPose(to_frame,t,input_pose,input_pose.header.frame_id,target_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
    }

    return target_pose;
}



//input parsing for pushing back points onto a trajectory
void* push_back_feedback(void *input_ptr)
{

    char input;
    // cin.ignore(numeric_limits<streamsize>::max(),'\n');
    std::cout << "Use [Enter] to push back a pose" << '\n';
    std::cout << "Use x to execute the Trajectory" << '\n';
    input =std::cin.get();
    ROS_INFO("Adding Point");
    geometry_msgs::PoseStamped target_pose;

    if(left_pose_received)
    {

        //you may be asking yourself... why is this transform here?
        //it's becasue I use the frame of the pose to determine which side it's on.
        target_pose = transform_pose_simple(&left_pose, rd_->getLeftPalmFrame());
        pose_pub->publish(target_pose);
    }
    if(right_pose_received)
    {
        target_pose = transform_pose_simple(&right_pose, rd_->getRightPalmFrame());
        pose_pub->publish(target_pose);
    }

    if(input == 'x')
    {
        std_msgs::Int16 send_trajectory;
        send_trajectory.data = true;
        execute_pub->publish(send_trajectory);
    }

    left_pose_received = false;
    right_pose_received = false;
    planning = false;
    pose_received = false;
    execute = false;
    ROS_INFO("Exit input thread");
    pthread_exit(NULL);
}

//input parsing for sending through MoveIt!
void* acquire_feedback(void *input_ptr)
{

    char input;
    bool left_failed=false,right_failed=false;
    int plan_status=0;

    cout<<"Press enter to continue";
    cin.ignore(numeric_limits<streamsize>::max(),'\n');
    cout<<'\n';
    string planner_type;
    std::cout << "       Enter planner type and press enter to execute" << '\n';
    std::cout << "       1:RRTConnect  2:RRTstar  3:PRM  4:BKPIECE  default:SBL" << '\n';
    std::cout << "       " ;
    input =std::cin.get();
    planning = true;

    switch (input) {
    case '1':
        planner_type = "RRTConnectkConfigDefault";
        break;
    case '2':
        planner_type = "RRTstarkConfigDefault";
        break;
    case '3':
        planner_type = "PRMkConfigDefault";
        break;
    case '4':
        planner_type = "BKPIECEkConfigDefault";
        break;
    default:
        planner_type = "SBLkConfigDefault";
        break;
    }

    ROS_INFO_STREAM("Planning with "<<planner_type<<'\n');
    //Test the sleep duration if it is actually needed.
    ros::Duration(0.5).sleep();
    if(left_pose_received){
        left_pose = transform_pose_simple(&left_pose,VAL_COMMON_NAMES::WORLD_TF);
        left_failed = plan_trajectory("leftArm",left_pose, left_plan,planner_type); //0 on success
    }
    if(right_pose_received){
        right_pose = transform_pose_simple(&right_pose,VAL_COMMON_NAMES::WORLD_TF);
        right_failed = plan_trajectory("rightArm",right_pose, right_plan,planner_type);//0 on success
    }
    ros::Duration(0.5).sleep();


    if(left_failed)
    {
        ROS_ERROR("Planning left failed");
    }else{

        ROS_INFO("Send left to controller");
        left_armTraj_port->moveArmTrajectory(LEFT, left_plan->trajectory_.joint_trajectory);
        ros::Duration(0.5).sleep();
    }

    if(right_failed)
    {
        ROS_ERROR("Planning right failed");
    }else{
        ROS_INFO("Send right to controller");
        right_armTraj_port->moveArmTrajectory(RIGHT, right_plan->trajectory_.joint_trajectory);
        ros::Duration(0.5).sleep();
    }

    left_pose_received = false;
    right_pose_received = false;
    planning = false;
    pose_received = false;
    execute = false;
    ROS_INFO("Exit input thread");
    pthread_exit(NULL);
}



void feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if(!planning)
    {
        if(!feedback->marker_name.compare("left_"))
        {
            // ROS_INFO("left pose received");
            left_pose_received = true;
            left_pose.pose.position = feedback->pose.position;
            left_pose.pose.orientation = feedback->pose.orientation;
            left_pose.header = feedback->header;

        }else{
            // ROS_INFO("Right pose received");
            right_pose_received = true;
            right_pose.pose.position = feedback->pose.position;
            right_pose.pose.orientation = feedback->pose.orientation;
            right_pose.header = feedback->header;
        }

        if(!pose_received)
        {
            pose_received = true;
            //spawn thread to check for user input????
            if(push_back)
            {
                pthread_create(&input_thread, NULL, push_back_feedback, NULL);
            }else{
                pthread_create(&input_thread, NULL, acquire_feedback, NULL);
            }

        }
    }else{

        if(!execute)
        {
            execute = true;

        }
    }
}


Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;
    // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // marker.scale.x = marker.scale.y = marker.scale.z = 2;
    // marker.color.r = 0;
    // marker.color.g = 0;
    // marker.color.b = 0;
    // marker.color.a = 0;
    // marker.mesh_resource = "package://val_description/model/arms/palm_right.dae";
    // marker.mesh_use_embedded_materials = true;
    marker.type = Marker::CUBE;
    marker.scale.x =.04;
    marker.scale.y =.04;
    marker.scale.z =.04;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}
void make6DofMarker(std::string name,void (*callback)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &) ,
                    bool fixed, unsigned int interaction_mode, const tf::Vector3& position,
                    geometry_msgs::Quaternion palm_rotation, bool show_6dof )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.pose.orientation = palm_rotation;
    int_marker.scale = .3;

    int_marker.name = name;
    int_marker.description = "Control";

    // insert a box
    makeBoxControl(int_marker);
    if(int_marker.controls.empty()){
        int_marker.controls.resize(1);
    }
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if ( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        // int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server_port->insert(int_marker);
    server_port->setCallback( int_marker.name, callback);
    server_port->applyChanges();
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *server_port, int_marker.name );
}


//notes
//Operation for normal posing using MoveIt!
//run rosrun val_moveit_tasks val_pose
//roslaunch valkyrie_moveit_config move_group.launch
//sub to /val_pose/update in rviz, on an interactive_marker
//pose arm (left arm not supported by moveit at this time, val_pose does support)
//in terminal, type the number relating to the planner desired.
//press enter.

//operation for trajectory posing using IHMC interpolators
//run rosrun val_moveit_tasks val_pose 1
//run rosrun val_task1 val_circle (don't as why it's in there)
//pose arms.
//press enter in terminal.
//repeat posing/enter process for as many waypoints as you like.
//on last pose, include an x on the terminal as you press enter.
//the robot should move through the poses.

int main(int argc, char** argv){

    ros::init(argc, argv, "val_pose");
    ros::NodeHandle node_handle;
    rd_ = RobotDescription::getRobotDescription(node_handle);
    nh_port = &node_handle;
    pose_pub = new ros::Publisher();    // every new keyword should have a delete somewhere.
    // checkout the end of main
    execute_pub = new ros::Publisher();
    *pose_pub = nh_port->advertise<geometry_msgs::PoseStamped>("/val_pose/push_back", 10);
    *execute_pub = nh_port->advertise<std_msgs::Int16>("/val_pose/execute", 10);
    push_back = false;

    //arg c is never 0.
    //arg c is len(argv)
    //if argv only has one item (ie. nodename, then argc == 1)
    //ergo, latin, (argc-1 == 0)* = nodename.
    if(*argv[argc-1] == '1')
    {
        push_back = true;
    }

    interactive_markers::InteractiveMarkerServer server("val_pose");
    server_port = &server;
    ArmControlInterface left_armTraj(*nh_port);
    left_armTraj_port = &left_armTraj;
    ArmControlInterface right_armTraj(*nh_port);
    right_armTraj_port = &right_armTraj;
    // right_pose = (geometry_msgs::PoseStamped*)malloc(sizeof(geometry_msgs::PoseStamped));
    left_plan = new moveit::planning_interface::MoveGroup::Plan();
    right_plan = new moveit::planning_interface::MoveGroup::Plan();
    // make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, true,&server );
    /**************************************
     * ALWAYS START SPINNER IF USING MOVEIT
     *************************************/

    ros::AsyncSpinner spinner(1);
    spinner.start();

    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener_port = &listener;
    ros::Duration(0.2).sleep();
    geometry_msgs::Quaternion palm_rotation;

    try
    {
        ros::Time t = ros::Time::now();
        listener_port->waitForTransform(rd_->getRightPalmFrame(),VAL_COMMON_NAMES::WORLD_TF,t, ros::Duration(3.0));
        listener_port->lookupTransform(VAL_COMMON_NAMES::WORLD_TF, rd_->getRightPalmFrame(), t,transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return -1;
    }
    //    there is an easier way to do this
    tf::quaternionTFToMsg(transform.getRotation(), palm_rotation);
    //    palm_rotation.x = transform.getRotation().getAxis().getX();
    //    palm_rotation.y = transform.getRotation().getAxis().getY();
    //    palm_rotation.z = transform.getRotation().getAxis().getZ();
    //    palm_rotation.w = transform.getRotation().getW();
    make6DofMarker("right",&feedback, false, InteractiveMarkerControl::ROTATE_AXIS, transform.getOrigin(),palm_rotation, true );

    try
    {
        ros::Time t = ros::Time::now();
        listener_port->waitForTransform(rd_->getLeftPalmFrame(),VAL_COMMON_NAMES::WORLD_TF,t, ros::Duration(3.0));
        listener_port->lookupTransform(VAL_COMMON_NAMES::WORLD_TF, rd_->getLeftPalmFrame(), t,transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return -1;
    }
    //    there is an easier way to do this
    tf::quaternionTFToMsg(transform.getRotation(), palm_rotation);
    //    palm_rotation.x = transform.getRotation().getAxis().getX();
    //    palm_rotation.y = transform.getRotation().getAxis().getY();
    //    palm_rotation.z = transform.getRotation().getAxis().getZ();
    //    palm_rotation.w = transform.getRotation().getW();
    make6DofMarker("left",&feedback, false, InteractiveMarkerControl::ROTATE_AXIS, transform.getOrigin(),palm_rotation, true );

    if(push_back)
    {
        ROS_INFO("Node Started: Jog Arm; Push Back");
    }else{
        ROS_INFO("Node Started: Jog Arm; Execute");
    }
    // start the ROS main loop
    ros::spin();

    delete pose_pub;
    delete execute_pub;
    //delete everything else
}
