#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <val_control/robot_state.h>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "turn_handle");
    ros::NodeHandle nh;
    RobotStateInformer* stateInformer = RobotStateInformer::getRobotStateInformer(nh);

    //get current pose of hand
    geometry_msgs::Pose currentHandPose;
    stateInformer->getCurrentPose("RightPalm", currentHandPose);

    //generate points for circle
    float radius = .13,num_steps = 31,x,y,sub_angle; //planar distance between center and edge (meters?)
    float x_values[(int) num_steps],y_values[(int) num_steps];


    // start_pose = new geometry_msgs::PoseStamped();

    //I believe this is the SLD between the current gripper pose and the center of the arc.

    //these two are the start point of the arc
    x = 2.3;//center.pose.position.x;
    y = 1.04;//center.pose.position.y;
    std::cout << "center: " <<x<<" "<<y<< '\n';
    std::cout << "radius: " <<radius<< '\n';

    num_steps = num_steps*(angle/360.0);
    for (int i=0;i<num_steps;i++)
    {
      // sub_angle=(i/(num_steps-1)*deg2rad(angle));
      // x = 450+(radius*numpy.cos(62*.1))
      // y = -450+(radius*numpy.sin(62*.1))
      x_values[i]=x+(radius*cos((float)i*(6.2/num_steps)));
      y_values[i]=y+(radius*sin((float)i*(6.2/num_steps)));
      x = x_values[i];
      y = y_values[i];
      ROS_INFO("X value %0.2f, Y value %0.2f",x,y);

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


    //populate marker

    //publish marker
}
