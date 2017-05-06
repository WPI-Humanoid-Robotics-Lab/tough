#include <val_task1/move_handle.h>



move_handle::move_handle(ros::NodeHandle n) : nh_(n) {
    array_pub = nh_.advertise<visualization_msgs::MarkerArray>( "handle_path", 0 );
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}


void move_handle::createCircle(geometry_msgs::Point center, int side, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points)
{

  float radius = .12,num_steps = 20;
  //float hand_z_plane = 0;
  float dist = 0;

  if (planeCoeffs.size() != 4){
      ROS_INFO("Please check the plane coefficiants");
  }

  float a = planeCoeffs.at(0);
  float b = planeCoeffs.at(1);
  float c = planeCoeffs.at(2);
  float d = planeCoeffs.at(3);

  geometry_msgs::Pose cen;
  cen.position.x = center.x;
  cen.position.y = center.y;
  cen.position.z = center.z;


  geometry_msgs::Pose current_hand_pose;

  // get the hand frame
  if (side)
    robot_state_->getCurrentPose("rightMiddleFingerPitch1Link",current_hand_pose);
  else
   robot_state_->getCurrentPose("leftMiddleFingerPitch1Link",current_hand_pose);

  //hand_z_plane = (a*current_hand_pose.position.x  + b*current_hand_pose.position.y - d)/c ;
  //hand_z_plane = current_hand_pose.position.z - hand_z_plane;

  dist = fabs( a*current_hand_pose.position.x  + b*current_hand_pose.position.y + c*current_hand_pose.position.z  + d )/sqrt( pow(a,2) + pow(b,2) + pow(c,2) );
  ROS_INFO("%f\n",dist);
  ROS_INFO_STREAM(current_hand_pose.position);

  for (int i=0; i<num_steps; ++i)
  {
    // circle parametric equation
    geometry_msgs::Pose point;
    point.position.x = cen.position.x + (radius * cos((float)i*(2*M_PI/num_steps)));
    point.position.y = cen.position.y + (radius * sin((float)i*(2*M_PI/num_steps)));
    point.position.z = -(a*point.position.x  + b* point.position.y + (d - dist) )/c   ;

    points.push_back(point);
  }

  visulatize(points);
  ros::Duration(3).sleep();
}


void move_handle::visulatize(std::vector<geometry_msgs::Pose> &points)
{
  // visulation of the circle
  visualization_msgs::MarkerArray circle = visualization_msgs::MarkerArray();
  for (int i = 0; i < points.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "circle";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = points[i];
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    circle.markers.push_back(marker);
  }

  array_pub.publish( circle );
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
