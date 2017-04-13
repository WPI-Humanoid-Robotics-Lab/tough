#include "test_circle.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "test_circle");

    ros::NodeHandle *node_handle = new ros::NodeHandle();
    center_pub    = new ros::Publisher();
    edge_pub      = new ros::Publisher();
    execute_pub   = new ros::Publisher();
    *center_pub   = node_handle->advertise<geometry_msgs::PoseStamped>("/val_pose/push_back/center", 10);
    *edge_pub     = node_handle->advertise<geometry_msgs::PoseStamped>("/val_pose/push_back/edge", 10);
    *execute_pub  = node_handle->advertise<std_msgs::Int16>("/val_pose/execute", 10);


    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(0.2).sleep();

//approximate right hand start position for testing
    // position:
    //   x: 0.0361732393617
    //   y: -0.161804376091
    //   z: 0.0806415066177
    // orientation:
    //   x: -0.257522648502
    //   y: -0.0309345790064
    //   z: -0.237831410495
    //   w: 0.93603491255

    center_start_pose.pose.position.x     = 0.0361732393617;
    center_start_pose.pose.position.y     = -0.161804376091;
    center_start_pose.pose.position.z     = 0.0806415066177;
    center_start_pose.pose.orientation.x  = -0.257522648502;
    center_start_pose.pose.orientation.y  = -0.0309345790064;
    center_start_pose.pose.orientation.z  = -0.237831410495;
    center_start_pose.pose.orientation.w  =  0.93603491255;
    center_start_pose.header.seq          = 1;
    center_start_pose.header.stamp        = ros::Time::now();
    center_start_pose.header.frame_id     = VAL_COMMON_NAMES::R_PALM_TF;

    ROS_INFO("Test Circle Node Started");
    // pthread_create(&input_thread, NULL, input_thread_func, NULL);
    input_thread_func(NULL);
    // start the ROS main loop
    // ros::spin();

    ros::Duration(0.2).sleep();
//    delete everything here
    delete node_handle;
    delete center_pub;
    delete edge_pub;
    delete execute_pub;
}

//this says it's a thread func, but it's called as a return
void* input_thread_func(void *input_ptr)
{
  char input;
  std::string input_str;



  //spawn a new marker (hopefully in the right place)
  Val_Marker center_marker("test_circle","Center Pose", center_start_pose, &center_marker_callback );
  ROS_INFO("Place Center Marker, Press {Enter}");
  center_pose = center_start_pose;
  //this waits until both center pose received and user presses enter
  input =std::cin.get();
  //man, this is pretty stupid
  center_pub->publish(transform_pose_simple(&center_pose,VAL_COMMON_NAMES::R_PALM_TF));

// std::cout << "pub pose: " <<center_pose.pose.position.x<<" "<<center_pose.pose.position.y<< '\n';

  // edge_start_pose = center_pose;
  // edge_start_pose.pose.position.y = edge_start_pose.pose.position.y +.25;
  // edge_pose = edge_start_pose;
  //
  // Val_Marker edge_marker("test_circle","edge Pose", edge_start_pose,&edge_marker_callback );
  // ROS_INFO("Place Edge Marker, Press {Enter}");
  // input =std::cin.get();
  // edge_pub->publish(transform_pose_simple(&edge_pose,VAL_COMMON_NAMES::R_PALM_TF));

  std::cout << "Enter angle to move: " << '\n';
  std::getline(std::cin,input_str);
  execute_angle.data = std::stoi(input_str);
  execute_pub->publish(execute_angle);

  ROS_INFO("Exit Input Thread");
  input = input;
  return NULL;
  // pthread_exit(NULL);
}

void center_marker_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  center_pose_received = true;
  center_pose.pose.position = feedback->pose.position;
  center_pose.pose.orientation = feedback->pose.orientation;
  center_pose.header = feedback->header;
}

void edge_marker_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  edge_pose_received = true;
  edge_pose.pose.position = feedback->pose.position;
  edge_pose.pose.orientation = feedback->pose.orientation;
  edge_pose.header = feedback->header;
}
