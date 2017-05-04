#include <val_circle.h>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "val_circle");
  ros::NodeHandle nh;

  right_armTraj = new armTrajectory(nh);
  left_armTraj = new armTrajectory(nh);
  right_poses = new std::vector<geometry_msgs::PoseStamped*>;
  left_poses = new std::vector<geometry_msgs::PoseStamped*>;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();
  ros::Subscriber pose_sub    = nh.subscribe("/val_pose/push_back", 10, pose_callback);
  ros::Subscriber center_sub  = nh.subscribe("/val_pose/push_back/center", 10, center_callback);//center and edge have to be stamped with the armside to use.
  ros::Subscriber edge_sub    = nh.subscribe("/val_pose/push_back/edge", 10, edge_callback);
  ros::Subscriber go_sub      = nh.subscribe("/val_pose/execute",10,execute_callback);
  array_pub    = nh.advertise<visualization_msgs::MarkerArray>( "Circle_Array", 0 );
  listener = new tf::TransformListener();


  // Val_Marker::Val_Marker("val_circle","Center", geometry_msgs::PoseStamped start_position,marker_callback );



  ros::Duration(0.2).sleep();
  ROS_INFO("Val_Circle");
  ros::spin();


  delete right_armTraj;
  delete left_armTraj;
  delete right_poses;
  delete left_poses;
  delete listener;

  return 0;
}


geometry_msgs::PoseStamped transform_pose_simple(const geometry_msgs::PoseStamped *from_pose,std::string to_frame)
{
  geometry_msgs::PoseStamped input_pose;
  geometry_msgs::PoseStamped target_pose;
  input_pose.pose.position = from_pose->pose.position;
  input_pose.pose.orientation = from_pose->pose.orientation;
  input_pose.header = from_pose->header;

  try
  {
      tf::StampedTransform transform;
      ros::Time t = ros::Time::now();
      listener->waitForTransform(input_pose.header.frame_id, to_frame, t, ros::Duration(3.0));
      listener->transformPose(to_frame,t,input_pose,input_pose.header.frame_id,target_pose);
  }
  catch (tf::TransformException ex)
  {   ROS_WARN("%s",ex.what());
      return transform_pose_simple(from_pose,to_frame);}

  return target_pose;
}


void pose_callback(const geometry_msgs::PoseStamped& msg)
{
  geometry_msgs::PoseStamped* input_pose = new geometry_msgs::PoseStamped;
  input_pose->pose = msg.pose;
  input_pose->header = msg.header;
  if(!msg.header.frame_id.compare(VAL_COMMON_NAMES::L_PALM_TF))
  {
    ROS_INFO("Left pose received");
    left_poses->push_back(input_pose);
    left_pose_received = true;
    std::cout << "input_pose: " <<*input_pose<< '\n';
  }
  if(!msg.header.frame_id.compare(VAL_COMMON_NAMES::R_PALM_TF)){
    ROS_INFO("Right pose received");
    right_poses->push_back(input_pose);
    right_pose_received = true;
    std::cout << "input_pose: " <<*input_pose<< '\n';
  }
}



void center_callback(const geometry_msgs::PoseStamped& msg)
{
  center_pose.pose = msg.pose;
  center_pose.header = msg.header;
  center_pose_received = true;
  edge_pose_received = true;
}
void edge_callback(const geometry_msgs::PoseStamped& msg)
{
  edge_pose.pose = msg.pose;
  edge_pose.header = msg.header;
  edge_pose_received = true;
}

//callback for execute commands.
//if I've received pushback requests for the left and the right,
//  it will execute those trajectories.
//if I've receieved a center_pose pushed back,
//  it will execute circle at the angle in the message.

void execute_callback(const std_msgs::Int16& msg)
{
  if(msg.data)
  {

    if(left_pose_received)
    {
      ROS_INFO("Received Left Execute Request");
      std::vector<armTrajectory::armTaskSpaceData>* arm_data_vector = generate_task_space_data(left_poses,armSide::LEFT);
      // moveArmInTaskSpace(std::vector &arm_data, int baseForControl) baseForControl defaults to chest
      left_armTraj->moveArmInTaskSpace(*arm_data_vector);
      left_poses->clear();
      arm_data_vector->clear();
      left_pose_received = false;
      // delete left_poses;
      // delete arm_data_vector;
    }
    if(right_pose_received)
    {
      ROS_INFO("Received Right Execute Request");
      std::vector<armTrajectory::armTaskSpaceData>* arm_data_vector = generate_task_space_data(right_poses,armSide::RIGHT);
      // moveArmInTaskSpace(std::vector &arm_data, int baseForControl) baseForControl defaults to chest
      right_armTraj->moveArmInTaskSpace(*arm_data_vector);
      right_poses->clear();
      arm_data_vector->clear();
      right_pose_received = false;
      // delete right_poses;
      // delete arm_data_vector;
    }
    //edge_pose_received overridden because I'm going to pull the curr_pose
    if(center_pose_received && edge_pose_received)
    {
      ROS_INFO("Received Circle Execute Request");
      execute_angle = msg.data;


      std::vector<geometry_msgs::PoseStamped*>* circle_poses = generate_circle_poses(center_pose, execute_angle);
      std::vector<armTrajectory::armTaskSpaceData>* arm_data_vector;
      //the pose has to be stamped with the side to this point.
      //ergo, latin, poses published to the val_circle have to be stamped with the arm_side to use.
      if(center_pose.header.frame_id == VAL_COMMON_NAMES::R_PALM_TF)
      {
        arm_data_vector = generate_task_space_data(circle_poses,armSide::RIGHT);
        right_armTraj->moveArmInTaskSpace(*arm_data_vector);


      }else{
        arm_data_vector = generate_task_space_data(circle_poses,armSide::LEFT);
        left_armTraj->moveArmInTaskSpace(*arm_data_vector); //poses are transformed here.
      }

      circle_poses->clear();
      arm_data_vector->clear();
      delete circle_poses;
      delete arm_data_vector;

      execute_angle = 0;
      center_pose_received = false;
      edge_pose_received =false;

    }
      ROS_INFO("Execute Order 66");
  }
}


//generates a semicircle from the center, edge, and theta.
//poses must be stampeed with the armside to use. default left.
vector<geometry_msgs::PoseStamped*>* generate_circle_poses(geometry_msgs::PoseStamped& center, int angle)
{
  float radius = .12,num_steps = 20,x,y,sub_angle; //planar distance between center and edge (meters?)
  float x_values[(int) num_steps],y_values[(int) num_steps];
  std::vector<geometry_msgs::PoseStamped*> *return_arc_poses;
  return_arc_poses = new std::vector<geometry_msgs::PoseStamped*>();
  geometry_msgs::PoseStamped *circle_pose;

  // start_pose = new geometry_msgs::PoseStamped();

  //I believe this is the SLD between the current gripper pose and the center of the arc.
  //radius = std::sqrt(center.pose.position.x * center.pose.position.x + center.pose.position.y * center.pose.position.y);

  //these two are the start point of the arc
  x = center.pose.position.x;
  y = center.pose.position.y;
  std::cout << "center: " <<x<<" "<<y<< '\n';
  std::cout << "radius: " <<radius<< '\n';

  num_steps = num_steps*(angle/360.0);
  for (int i=0;i<num_steps;i++)
  {
    // sub_angle=(i/(num_steps-1)*deg2rad(angle));
    // x = 450+(radius*numpy.cos(62*.1))
    // y = -450+(radius*numpy.sin(62*.1))
    x_values[i]=center.pose.position.x+(radius*cos((float)i*(6.2/num_steps)));
    y_values[i]=center.pose.position.y+(radius*sin((float)i*(6.2/num_steps)));
    x = x_values[i];
    y = y_values[i];

  }


//this creates poses in the handframe.
//It's my understanding, thus, that each of the values in it
//  are relative to the current pose of the active_hand.
//  I'm not sure about the orientation, though; idk how to set that.
    for (int i = 0; i < num_steps; i++) {
      circle_pose = new geometry_msgs::PoseStamped();
      circle_pose->header = center.header;
      circle_pose->pose.position.x = x_values[i];
      circle_pose->pose.position.y = y_values[i];
      circle_pose->pose.position.z = 0;
      circle_pose->pose.orientation = center.pose.orientation;
      return_arc_poses->push_back(circle_pose);
      // std::cout << circle_pose->pose.position << '\n';
    }


    visualization_msgs::MarkerArray circle = visualization_msgs::MarkerArray();
    for (int i = 0; i < num_steps; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = VAL_COMMON_NAMES::R_PALM_TF;
        marker.header.stamp = ros::Time();
        marker.ns = "circle";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = return_arc_poses->at(i)->pose;
        marker.scale.x = .01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = .6; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        circle.markers.push_back(marker);
        // std::cout << marker.pose.position << '\n';
      }

    array_pub.publish( circle );

    return return_arc_poses;
}


//caution, allocates task_space_data that needs to be freed eventually!
//armside is used to determine which side to use.
//transforms the poses to the worldframe regardless.
std::vector<armTrajectory::armTaskSpaceData>* generate_task_space_data(std::vector<geometry_msgs::PoseStamped*>* input_poses, armSide input_side, float desired_time)
{
 std::vector<armTrajectory::armTaskSpaceData> *arm_data_vector = new std::vector<armTrajectory::armTaskSpaceData>();
  // for(auto input_pose : *input_poses)

  int num_poses = input_poses->size();
  for(int i=0;i<num_poses;i++)
  {
    auto input_pose=input_poses->at(i);
    armTrajectory::armTaskSpaceData* task_space_data = new armTrajectory::armTaskSpaceData();
    task_space_data->side = input_side;
    task_space_data->pose = transform_pose_simple(input_pose,VAL_COMMON_NAMES::WORLD_TF).pose;
    task_space_data->time = (float)((float)i/(float)num_poses)*(float)desired_time; //what xyzzy

    arm_data_vector->push_back(*task_space_data);
  }
  return arm_data_vector;
}
