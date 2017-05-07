
#include <val_task1/move_handle.h>



move_handle::move_handle(ros::NodeHandle n) : nh_(n) {
    right_armTraj = new armTrajectory(nh_);
    left_armTraj = new armTrajectory(nh_);
    array_pub = nh_.advertise<visualization_msgs::MarkerArray>( "handle_path", 10, true );
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);



}


void move_handle::createCircle(geometry_msgs::PoseStamped center, int side, float a, float b, float c, float d)
{
  float radius = .12,num_steps = 10;
  float hand_z_plane = 0;
  float dist = 0;
  armSide input_side;

  geometry_msgs::Pose finger_pose;
  geometry_msgs::Pose hand_pose;

  // get the hand frame
  if (side){
    robot_state_->getCurrentPose("rightMiddleFingerPitch1Link",finger_pose);
    robot_state_->getCurrentPose(VAL_COMMON_NAMES::R_PALM_TF,hand_pose);
    input_side = armSide::RIGHT;
  }
  else{
   robot_state_->getCurrentPose("leftMiddleFingerPitch1Link",finger_pose);
   robot_state_->getCurrentPose(VAL_COMMON_NAMES::L_PALM_TF,hand_pose);
   input_side = armSide::LEFT;
   }

  //hand_z_plane = (a*current_hand_pose.position.x  + b*current_hand_pose.position.y - d)/c ;
  //hand_z_plane = current_hand_pose.position.z - hand_z_plane;

  dist = fabs( a*finger_pose.position.x  + b*finger_pose.position.y + c*finger_pose.position.z  + d )/sqrt( pow(a,2) + pow(b,2) + pow(c,2) );
  ROS_INFO("%f\n",dist);
  std::vector<geometry_msgs::PoseStamped> points;
  for (int i=0; i<num_steps; i++)
  {
    // circle parametric equation
    geometry_msgs::PoseStamped point;
    point.pose.position.x = center.pose.position.x + (radius * cos((float)i*(2*M_PI/num_steps) - .7*M_PI ));
    point.pose.position.y = center.pose.position.y + (radius * sin((float)i*(2*M_PI/num_steps) - .7*M_PI ));

    point.pose.position.z = -(a*point.pose.position.x  + b* point.pose.position.y + (d - dist+.05) )/c   ;
    points.push_back(point);
  }
  visulatize(points);
  follow_path(points, input_side,finger_pose);
}

void move_handle::follow_path(std::vector<geometry_msgs::PoseStamped>& points, armSide input_side, geometry_msgs::Pose hand)
{

    std::vector<armTrajectory::armTaskSpaceData> *arm_data_vector = new std::vector<armTrajectory::armTaskSpaceData>();
     // for(auto input_pose : *input_poses)
    float desired_time = 10.0;
    int num_poses = points.size();
    for(int i=0;i<num_poses;i++)
    {
        auto input_pose = points.at(i);
        armTrajectory::armTaskSpaceData* task_space_data = new armTrajectory::armTaskSpaceData();
        task_space_data->side = input_side;
        task_space_data->pose.position = input_pose.pose.position;
        task_space_data->pose.orientation = hand.orientation;
        task_space_data->time = (i/num_poses)*desired_time; //what xyzzy
        arm_data_vector->push_back(*task_space_data);
        //ROS_INFO_STREAM(task_space_data->pose);
    }
    if ( input_side == armSide::RIGHT )
        right_armTraj->moveArmInTaskSpace(*arm_data_vector);
    else
        left_armTraj->moveArmInTaskSpace(*arm_data_vector);

    ROS_INFO("Moved Handle");

}

void move_handle::visulatize(std::vector<geometry_msgs::PoseStamped> &points)
{
  // visulation of the circle
  visualization_msgs::MarkerArray circle = visualization_msgs::MarkerArray();

  visualization_msgs::Marker marker;
  marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
  marker.header.stamp = ros::Time();
  marker.ns = "circle";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = points[0].pose;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 0.6;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(0);
  circle.markers.push_back(marker);


  for (int i = 1; i < points.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "circle";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = points[i].pose;
    ROS_INFO_STREAM(i<<"pose"<<points[i].pose.position);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);
    circle.markers.push_back(marker);
  }

  array_pub.publish( circle );

  ROS_INFO("vis");
}





// int main(int argc, char** argv){
//
//   ros::init(argc, argv, "panel_detector");
//   ros::NodeHandle nh;
//   array_pub    = nh.advertise<visualization_msgs::MarkerArray>( "Circle_Array", 0 );
//
//   ros::Rate loop_rate(10);
//
//   geometry_msgs::PoseStamped center;
//   center.pose.position.x = 3.12;
//   center.pose.position.y = 0.814;
//   center.pose.position.z = 0.84;
//   center.pose.orientation.x  = 0;
//   center.pose.orientation.y  = 0;
//   center.pose.orientation.z  = 0;
//   center.pose.orientation.w  = 1;
//
//   std::vector<geometry_msgs::PoseStamped> points;
//
//   while(ros::ok())
//   {
//     createCircle(center, points);
//
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }
