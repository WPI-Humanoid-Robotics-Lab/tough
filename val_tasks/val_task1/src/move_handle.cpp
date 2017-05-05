
#include <val_task1/move_handle.h>



move_handle::move_handle(ros::NodeHandle n) : nh_(n){
      array_pub = nh_.advertise<visualization_msgs::MarkerArray>( "handle_path", 0 );
}


void move_handle::createCircle(geometry_msgs::PoseStamped center, float a, float b, float c, float d)
{
  float radius = .12,num_steps = 20;

  std::vector<geometry_msgs::PoseStamped> points;
  for (int i=0; i<num_steps; i++)
  {
    // circle parametric equation
    geometry_msgs::PoseStamped point;
    point.pose.position.x = center.pose.position.x + (radius * cos((float)i*(2*M_PI/num_steps)));
    point.pose.position.y = center.pose.position.y + (radius * sin((float)i*(2*M_PI/num_steps)));
    point.pose.position.z = (a*point.pose.position.x  + b* point.pose.position.y - (d-.7))/c ;
    points.push_back(point);
  }
  visulatize(points);
}


void move_handle::visulatize(std::vector<geometry_msgs::PoseStamped> &points)
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
    marker.pose = points[i].pose;
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
