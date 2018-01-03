#include "val_gui/val_marker.h"

int transform_attempts = 0;
using namespace visualization_msgs;
geometry_msgs::PoseStamped transform_pose_simple(const geometry_msgs::PoseStamped *from_pose,std::string to_frame)
{
  geometry_msgs::PoseStamped input_pose;
  geometry_msgs::PoseStamped target_pose;
  input_pose.pose.position = from_pose->pose.position;
  input_pose.pose.orientation = from_pose->pose.orientation;
  input_pose.header = from_pose->header;

  tf::TransformListener *listener = new tf::TransformListener();

  ros::Duration(0.2).sleep();
  try
  {
      tf::StampedTransform transform;
      ros::Time t = ros::Time::now();
      listener->waitForTransform(input_pose.header.frame_id, to_frame, t, ros::Duration(3.0));
      listener->transformPose(to_frame,t,input_pose,input_pose.header.frame_id,target_pose);
      //
      // ROS_INFO("Transform complete, T: ");
      //
      // std::cout << "input_frame: " <<input_pose.header.frame_id<< '\n';
      // std::cout << "outpt_frame: " <<target_pose.header.frame_id<< '\n';
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s",ex.what());
    ++transform_attempts;
    if(transform_attempts <3){
      return transform_pose_simple(from_pose,to_frame);
    }else{
    ROS_ERROR("ABORTING TRANSFORMATION, CANNOT TRANSFORM");
      exit(-1);
      }
  }

  return target_pose;
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
void make6DofMarker(std::string name,interactive_markers::InteractiveMarkerServer *i_marker_server,
                    void (*callback)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &), const tf::Vector3& position,
                    geometry_msgs::Quaternion palm_rotation,std::string frame,interactive_markers::MenuHandler *menu_handler)
{
  bool fixed = false;
  unsigned int interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  bool show_6dof = true;
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame;//"world";
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

  i_marker_server->insert(int_marker);
  i_marker_server->setCallback( int_marker.name, callback);
  i_marker_server->applyChanges();
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler->apply( *i_marker_server, int_marker.name );
}


//the pose sent in will be transformed to the world frame.
Val_Marker::Val_Marker(string node_name,std::string name, geometry_msgs::PoseStamped& start_pose,void (*callback)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &) )
{

  i_marker_server = new interactive_markers::InteractiveMarkerServer(node_name);
  menu_handler = new interactive_markers::MenuHandler();
  ros::Duration(0.2).sleep();
  geometry_msgs::PoseStamped world_pose = transform_pose_simple(&start_pose,VAL_COMMON_NAMES::WORLD_TF);

  tf::Vector3 position;
  position.setX(tfScalar(world_pose.pose.position.x));
  position.setY(tfScalar(world_pose.pose.position.y));
  position.setZ(tfScalar(world_pose.pose.position.z));

  make6DofMarker(name,i_marker_server,callback, position,world_pose.pose.orientation,world_pose.header.frame_id,menu_handler );
}

Val_Marker::~Val_Marker()
{
    delete i_marker_server;
    delete menu_handler;
}
